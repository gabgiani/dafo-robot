from __future__ import annotations

import argparse
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import mujoco
import mujoco.viewer
import numpy as np
import torch

from external_control import UdpExternalControl

ROOT = Path(__file__).resolve().parent
ASSET_ROOT = ROOT / "third_party" / "unitree_rl_gym"
XML_PATH = ASSET_ROOT / "resources" / "robots" / "g1_description" / "scene.xml"
POLICY_PATH = ASSET_ROOT / "deploy" / "pre_train" / "g1" / "motion.pt"

_HELP_TEXT = (
    "Controles: W/S avance +/- | A/D giro izq/der | Espacio centra avance y giro | "
    "R reinicia | P pausa | O reanuda | C captura lo que ve el robot (RGB + deteccion "
    "por geometria, guarda en /tmp/robot_pov.png). La politica sigue sosteniendo el "
    "equilibrio aunque no muevas nada (avance=0 no es lo mismo que pausar)."
)


@dataclass
class RLPolicyConfig:
    # Valores tomados directo de unitree_rl_gym/deploy/deploy_mujoco/configs/g1.yaml
    simulation_dt: float = 0.002
    control_decimation: int = 10
    kps: tuple[float, ...] = (100, 100, 100, 150, 40, 40, 100, 100, 100, 150, 40, 40)
    kds: tuple[float, ...] = (2, 2, 2, 4, 2, 2, 2, 2, 2, 4, 2, 2)
    default_angles: tuple[float, ...] = (-0.1, 0.0, 0.0, 0.3, -0.2, 0.0, -0.1, 0.0, 0.0, 0.3, -0.2, 0.0)
    ang_vel_scale: float = 0.25
    dof_pos_scale: float = 1.0
    dof_vel_scale: float = 0.05
    action_scale: float = 0.25
    cmd_scale: tuple[float, float, float] = (2.0, 2.0, 0.25)
    num_actions: int = 12
    num_obs: int = 47
    gait_period: float = 0.8


def get_gravity_orientation(quat: np.ndarray) -> np.ndarray:
    qw, qx, qy, qz = quat
    gravity_orientation = np.zeros(3, dtype=np.float32)
    gravity_orientation[0] = 2 * (-qz * qx + qw * qy)
    gravity_orientation[1] = -2 * (qz * qy + qw * qx)
    gravity_orientation[2] = 1 - 2 * (qw * qw + qz * qz)
    return gravity_orientation


def pd_control(
    target_q: np.ndarray,
    q: np.ndarray,
    kp: np.ndarray,
    target_dq: np.ndarray,
    dq: np.ndarray,
    kd: np.ndarray,
) -> np.ndarray:
    return (target_q - q) * kp + (target_dq - dq) * kd


class G1RLWalker:
    """Corre la politica RL pre-entrenada de unitree_rl_gym (sim2sim) sobre MuJoCo.

    A diferencia de interactive_unitree.py (controlador de marcha hecho a mano),
    aqui el balance y el centro de masa los resuelve la politica entrenada,
    no un heuristico ni fuerzas externas aplicadas a la pelvis.
    """

    def __init__(self, control_host: str, control_port: int | None, xml_path: Path | None = None):
        self.cfg = RLPolicyConfig()
        self.model = mujoco.MjModel.from_xml_path(str(xml_path or XML_PATH))
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = self.cfg.simulation_dt

        self.policy = torch.jit.load(str(POLICY_PATH))
        self.policy.eval()

        self._default_angles = np.array(self.cfg.default_angles, dtype=np.float32)
        self._kps = np.array(self.cfg.kps, dtype=np.float32)
        self._kds = np.array(self.cfg.kds, dtype=np.float32)
        self._cmd_scale = np.array(self.cfg.cmd_scale, dtype=np.float32)
        self._pelvis_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
        # El robot siempre es el primer cuerpo del arbol cinematico (qpos/qvel empiezan
        # con su floating base + 12 piernas); cualquier objeto extra en la escena
        # (cajas, estantes) queda despues, por eso acotamos el slice en vez de usar [7:]/[6:].
        n = self.cfg.num_actions
        self._qpos_leg_slice = slice(7, 7 + n)
        self._qvel_leg_slice = slice(6, 6 + n)

        # Si el modelo tiene mas actuadores que piernas (cintura/brazos/manos, ej.
        # g1_29dof_with_hand.xml), esos se sostienen con un PD propio a una pose
        # neutra -- la politica RL solo maneja las 12 piernas, nunca supo que existen.
        self._num_upper = self.model.nu - n
        if self._num_upper > 0:
            self._qpos_upper_slice = slice(7 + n, 7 + n + self._num_upper)
            self._qvel_upper_slice = slice(6 + n, 6 + n + self._num_upper)
            self._upper_body_target = np.zeros(self._num_upper, dtype=np.float32)
            if self._num_upper == 31:
                # 3 cintura + (7 brazo + 7 mano) x2 lados, en el orden real del XML.
                self._upper_kp = np.array(
                    [80, 80, 80] + [30] * 7 + [3] * 7 + [30] * 7 + [3] * 7, dtype=np.float32
                )
                self._upper_kd = np.array(
                    [2, 2, 2] + [1] * 7 + [0.05] * 7 + [1] * 7 + [0.05] * 7, dtype=np.float32
                )
            else:
                self._upper_kp = np.full(self._num_upper, 30.0, dtype=np.float32)
                self._upper_kd = np.full(self._num_upper, 1.0, dtype=np.float32)

        self.cmd = np.zeros(3, dtype=np.float32)  # [avance, lateral, giro] en [-1, 1]
        self.paused = False
        self.action = np.zeros(self.cfg.num_actions, dtype=np.float32)
        self.target_dof_pos = self._default_angles.copy()
        self.counter = 0
        self.axis_step = 0.2
        self._pov_cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "scene_cam")
        self._pov_renderer: mujoco.Renderer | None = None

        self.control_server = (
            UdpExternalControl(control_host, control_port) if control_port else None
        )

    def _print_status(self, message: str) -> None:
        print(f"[rl] {message}")

    def reset(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        self.action[:] = 0.0
        self.target_dof_pos[:] = self._default_angles
        self.counter = 0

    def _apply_external_payload(self, payload: dict, sender: tuple[str, int]) -> None:
        advance = payload.get("advance")
        if advance is not None:
            self.cmd[0] = max(-1.0, min(1.0, float(advance)))

        lateral = payload.get("lateral")
        if lateral is not None:
            self.cmd[1] = max(-1.0, min(1.0, float(lateral)))

        turn = payload.get("turn")
        if turn is not None:
            self.cmd[2] = max(-1.0, min(1.0, float(turn)))

        if payload.get("center"):
            self.cmd[:] = 0.0

        if "paused" in payload:
            self.paused = bool(payload["paused"])
            self._print_status("pausado" if self.paused else "reanudado")

        if payload.get("reset"):
            self.reset()
            self._print_status("simulacion reiniciada")

        if payload.get("capture"):
            self._capture_pov("/tmp/robot_pov.png")

        if payload:
            self._print_status(f"comando externo desde {sender[0]}:{sender[1]}")

    def _poll_external_control(self) -> None:
        if self.control_server is None:
            return
        for command in self.control_server.poll():
            try:
                self._apply_external_payload(command.payload, command.sender)
            except (TypeError, ValueError) as exc:
                self._print_status(f"comando externo invalido: {exc}")

    def _print_drive_status(self) -> None:
        self._print_status(
            f"avance={self.cmd[0]:+.2f} lateral={self.cmd[1]:+.2f} giro={self.cmd[2]:+.2f}"
        )

    def on_key(self, keycode: int) -> None:
        if keycode == 32:  # espacio
            self.cmd[:] = 0.0
            self._print_status("joystick centrado (la politica sigue de pie)")
            return

        if not (32 < keycode <= 126):
            return
        key = chr(keycode).lower()

        if key == "w":
            self.cmd[0] = max(-1.0, min(1.0, self.cmd[0] + self.axis_step))
            self._print_drive_status()
        elif key == "s":
            self.cmd[0] = max(-1.0, min(1.0, self.cmd[0] - self.axis_step))
            self._print_drive_status()
        elif key == "a":
            self.cmd[2] = max(-1.0, min(1.0, self.cmd[2] + self.axis_step))
            self._print_drive_status()
        elif key == "d":
            self.cmd[2] = max(-1.0, min(1.0, self.cmd[2] - self.axis_step))
            self._print_drive_status()
        elif key == "r":
            self.cmd[:] = 0.0
            self.reset()
            self._print_status("simulacion reiniciada")
        elif key == "p":
            self.paused = True
            self._print_status("pausado")
        elif key == "o":
            self.paused = False
            self._print_status("reanudado")
        elif key == "c":
            self._capture_pov("/tmp/robot_pov.png")

    def _run_policy(self) -> None:
        cfg = self.cfg
        default_angles = self._default_angles

        qj = (self.data.qpos[self._qpos_leg_slice] - default_angles) * cfg.dof_pos_scale
        dqj = self.data.qvel[self._qvel_leg_slice] * cfg.dof_vel_scale
        gravity_orientation = get_gravity_orientation(self.data.qpos[3:7])
        omega = self.data.qvel[3:6] * cfg.ang_vel_scale

        count = self.counter * cfg.simulation_dt
        phase = (count % cfg.gait_period) / cfg.gait_period
        sin_phase = np.sin(2 * np.pi * phase)
        cos_phase = np.cos(2 * np.pi * phase)

        n = cfg.num_actions
        obs = np.zeros(cfg.num_obs, dtype=np.float32)
        obs[:3] = omega
        obs[3:6] = gravity_orientation
        obs[6:9] = self.cmd * self._cmd_scale
        obs[9 : 9 + n] = qj
        obs[9 + n : 9 + 2 * n] = dqj
        obs[9 + 2 * n : 9 + 3 * n] = self.action
        obs[9 + 3 * n : 9 + 3 * n + 2] = (sin_phase, cos_phase)

        with torch.no_grad():
            obs_tensor = torch.from_numpy(obs).unsqueeze(0)
            action = self.policy(obs_tensor).detach().numpy().squeeze()

        self.action = action
        self.target_dof_pos = action * cfg.action_scale + default_angles

    def _step_physics(self) -> None:
        tau = pd_control(
            self.target_dof_pos,
            self.data.qpos[self._qpos_leg_slice],
            self._kps,
            np.zeros_like(self._kds),
            self.data.qvel[self._qvel_leg_slice],
            self._kds,
        )
        self.data.ctrl[: self.cfg.num_actions] = tau

        if self._num_upper > 0:
            upper_tau = pd_control(
                self._upper_body_target,
                self.data.qpos[self._qpos_upper_slice],
                self._upper_kp,
                np.zeros_like(self._upper_kd),
                self.data.qvel[self._qvel_upper_slice],
                self._upper_kd,
            )
            self.data.ctrl[self.cfg.num_actions :] = upper_tau

        mujoco.mj_step(self.model, self.data)
        self.counter += 1
        if self.counter % self.cfg.control_decimation == 0:
            self._run_policy()

    def _update_pov_camera(self) -> None:
        """Reposiciona la camara de escena para que siga la cabeza del robot
        (posicion de la pelvis + offset fijo hacia adelante/arriba, con el yaw
        del robot pero sin heredar el balanceo de la marcha en pitch/roll)."""
        pelvis_pos = self.data.xpos[self._pelvis_id]
        qw, qx, qy, qz = self.data.qpos[3:7]
        yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        cos_yaw, sin_yaw = np.cos(yaw), np.sin(yaw)
        r_yaw = np.array([[cos_yaw, -sin_yaw, 0.0], [sin_yaw, cos_yaw, 0.0], [0.0, 0.0, 1.0]])

        local_offset = np.array([0.15, 0.0, 0.55])  # adelante, lateral, arriba desde la pelvis
        cam_pos = pelvis_pos + r_yaw @ local_offset

        local_x = np.array([0.0, -1.0, 0.0])
        local_y = np.array([0.35, 0.0, 0.94])
        world_x = r_yaw @ local_x
        world_y = r_yaw @ local_y
        world_z = np.cross(world_x, world_y)
        cam_mat = np.column_stack([world_x, world_y, world_z])

        cam_quat = np.zeros(4)
        mujoco.mju_mat2Quat(cam_quat, cam_mat.flatten())

        self.model.cam_pos[self._pov_cam_id] = cam_pos
        self.model.cam_quat[self._pov_cam_id] = cam_quat
        mujoco.mj_kinematics(self.model, self.data)
        mujoco.mj_camlight(self.model, self.data)

    def _capture_pov(self, save_path: str) -> None:
        if self._pov_cam_id < 0:
            self._print_status("esta escena no tiene camara 'scene_cam', no puedo capturar")
            return

        from locate_objects_lidar import (
            camera_intrinsics,
            depth_to_point_cloud,
            segment_objects_above_floor,
        )

        self._update_pov_camera()

        width, height = 640, 480
        if self._pov_renderer is None:
            self._pov_renderer = mujoco.Renderer(self.model, height=height, width=width)
        renderer = self._pov_renderer

        renderer.update_scene(self.data, camera=self._pov_cam_id)
        rgb = renderer.render().copy()
        renderer.enable_depth_rendering()
        renderer.update_scene(self.data, camera=self._pov_cam_id)
        depth = renderer.render().copy()
        renderer.disable_depth_rendering()

        cam_pos = self.data.cam_xpos[self._pov_cam_id].copy()
        cam_mat = self.data.cam_xmat[self._pov_cam_id].reshape(3, 3).copy()
        fx, fy, cx, cy = camera_intrinsics(self.model, self._pov_cam_id, width, height)
        points_world = depth_to_point_cloud(depth, cam_pos, cam_mat, fx, fy, cx, cy)
        objects = segment_objects_above_floor(points_world, rgb, depth)

        import cv2
        from PIL import Image

        debug = rgb.copy()
        self._print_status(f"capturando vista del robot: {len(objects)} objeto(s) detectado(s) por geometria")
        for i, obj in enumerate(objects):
            u, v = int(obj.pixel_centroid[0]), int(obj.pixel_centroid[1])
            cv2.drawMarker(debug, (u, v), (0, 255, 0), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
            self._print_status(
                f"  [{i}] posicion_3d={obj.world_position.round(3)} rgb={obj.mean_rgb} px={obj.pixel_count}"
            )
        Image.fromarray(debug).save(save_path)
        self._print_status(f"imagen guardada en {save_path}")

    def _configure_camera(self, viewer: Any) -> None:
        # Camara "tracking": sigue automaticamente a la pelvis cada cuadro,
        # asi el robot no se sale de cuadro mientras camina.
        viewer.cam.trackbodyid = self._pelvis_id
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        viewer.cam.distance = 3.0
        viewer.cam.azimuth = 145
        viewer.cam.elevation = -14

    def run(self, max_seconds: float | None = None) -> None:
        wall_start = time.perf_counter()
        self.reset()
        with mujoco.viewer.launch_passive(
            self.model,
            self.data,
            key_callback=self.on_key,
            show_left_ui=True,
            show_right_ui=True,
        ) as viewer:
            self._print_status(_HELP_TEXT)
            if self.control_server is not None:
                host, port = self.control_server.address
                self._print_status(
                    f"control externo UDP en {host}:{port} | usa send_unitree_command.py"
                )
            with viewer.lock():
                self._configure_camera(viewer)
            viewer.sync()

            while viewer.is_running():
                step_start = time.perf_counter()
                self._poll_external_control()

                if not self.paused:
                    self._step_physics()

                viewer.sync()

                if max_seconds is not None and time.perf_counter() - wall_start >= max_seconds:
                    break

                remaining = self.model.opt.timestep - (time.perf_counter() - step_start)
                if remaining > 0:
                    time.sleep(remaining)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Corre la politica RL pre-entrenada de unitree_rl_gym para G1 sobre MuJoCo (sim2sim)."
    )
    parser.add_argument("--control-host", default="127.0.0.1")
    parser.add_argument(
        "--control-port",
        type=int,
        default=47001,
        help="Puerto UDP para control externo (avance/lateral/turn). Usa 0 para desactivarlo.",
    )
    parser.add_argument(
        "--max-seconds",
        type=float,
        help="Cierra el viewer automaticamente tras N segundos. Util para validar el arranque.",
    )
    parser.add_argument(
        "--scene",
        help=(
            "Ruta opcional a un XML de escena alternativo (por ejemplo "
            "scenes/g1_warehouse_scene.xml con cajas y estante). Por defecto usa el "
            "scene.xml plano de unitree_rl_gym."
        ),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    xml_path = Path(args.scene).expanduser().resolve() if args.scene else XML_PATH
    if not xml_path.exists() or not POLICY_PATH.exists():
        print(
            "Error: no encontre los assets necesarios. "
            f"Esperaba {xml_path} y {POLICY_PATH}.",
            flush=True,
        )
        return 1

    walker = G1RLWalker(
        args.control_host,
        None if args.control_port == 0 else args.control_port,
        xml_path=xml_path,
    )
    walker.run(max_seconds=args.max_seconds)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
