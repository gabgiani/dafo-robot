from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Any

import mujoco
import mujoco.viewer

from external_control import UdpExternalControl


_HELP_TEXT = (
    "Controles: espacio pausa/reanuda | W/S avance +/- | A/D giro izq/der | "
    "X centrado | R reinicia | J/K bajan/suben amplitud | N/M bajan/suben frecuencia"
)


@dataclass
class GaitTuning:
    hip_pitch: float = 0.11
    knee: float = 0.18
    ankle_pitch: float = 0.10
    hip_roll: float = 0.016
    shoulder_pitch: float = 0.08
    frequency_hz: float = 0.8
    amplitude_scale: float = 0.6
    swing_height: float = 0.03


class PassiveUnitreeSimulator:
    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        keyframe: str | None,
        control_server: UdpExternalControl | None = None,
    ):
        self.model = model
        self.data = data
        self.keyframe = keyframe
        self.control_server = control_server
        self.gait = GaitTuning()
        self.paused = False
        self.target_advance = 0.0
        self.target_turn = 0.0
        self.current_advance = 0.0
        self.current_turn = 0.0
        self._keyframe_id = self._resolve_keyframe_id(keyframe)
        self._pelvis_id = self._body_id("pelvis")
        self._base_ctrl = data.ctrl.copy()
        self._actuators = {
            name: self._actuator_id(name)
            for name in (
                "left_hip_pitch_joint",
                "right_hip_pitch_joint",
                "left_knee_joint",
                "right_knee_joint",
                "left_ankle_pitch_joint",
                "right_ankle_pitch_joint",
                "left_ankle_roll_joint",
                "right_ankle_roll_joint",
                "left_hip_roll_joint",
                "right_hip_roll_joint",
                "left_hip_yaw_joint",
                "right_hip_yaw_joint",
                "waist_pitch_joint",
                "waist_yaw_joint",
                "left_shoulder_pitch_joint",
                "right_shoulder_pitch_joint",
            )
        }
        self._foot_sites = {
            "left": self._site_id("left_foot"),
            "right": self._site_id("right_foot"),
        }
        self._neutral_foot_heights = self._capture_neutral_foot_heights()

    def _resolve_keyframe_id(self, keyframe: str | None) -> int:
        if not keyframe:
            return -1
        return mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, keyframe)

    def _actuator_id(self, name: str) -> int:
        actuator_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if actuator_id < 0:
            raise ValueError(f"No encontre el actuador '{name}'.")
        return actuator_id

    def _body_id(self, name: str) -> int:
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
        if body_id < 0:
            raise ValueError(f"No encontre el body '{name}'.")
        return body_id

    def _site_id(self, name: str) -> int:
        site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, name)
        if site_id < 0:
            raise ValueError(f"No encontre el site '{name}'.")
        return site_id

    def _capture_neutral_foot_heights(self) -> dict[str, float]:
        return {
            side: float(self.data.site_xpos[site_id][2])
            for side, site_id in self._foot_sites.items()
        }

    def reset(self) -> None:
        if self._keyframe_id >= 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, self._keyframe_id)
        else:
            mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        self._base_ctrl = self.data.ctrl.copy()
        self._neutral_foot_heights = self._capture_neutral_foot_heights()

    def _print_status(self, message: str) -> None:
        print(f"[viewer] {message}")

    def _coerce_float(self, payload: dict[str, Any], key: str) -> float | None:
        if key not in payload:
            return None
        value = payload[key]
        if isinstance(value, (int, float)):
            return float(value)
        raise ValueError(f"'{key}' debe ser numerico")

    def _coerce_bool(self, payload: dict[str, Any], key: str) -> bool | None:
        if key not in payload:
            return None
        value = payload[key]
        if isinstance(value, bool):
            return value
        raise ValueError(f"'{key}' debe ser booleano")

    def _char_from_key(self, keycode: int) -> str | None:
        if 32 <= keycode <= 126:
            return chr(keycode).lower()
        return None

    def _toggle_pause(self) -> None:
        self.paused = not self.paused
        self._print_status("pausado" if self.paused else "reanudado")

    def _clamp_axis(self, value: float) -> float:
        return max(-1.0, min(1.0, value))

    def _print_drive_status(self) -> None:
        self._print_status(
            f"avance={self.target_advance:+.2f} giro={self.target_turn:+.2f}"
        )

    def _adjust_advance(self, delta: float) -> None:
        self.target_advance = self._clamp_axis(self.target_advance + delta)
        self._print_drive_status()

    def _adjust_turn(self, delta: float) -> None:
        self.target_turn = self._clamp_axis(self.target_turn + delta)
        self._print_drive_status()

    def _center_drive(self) -> None:
        self.target_advance = 0.0
        self.target_turn = 0.0
        self._print_status("joystick centrado")

    def _set_drive_targets(self, advance: float | None = None, turn: float | None = None) -> None:
        changed = False
        if advance is not None:
            self.target_advance = self._clamp_axis(advance)
            changed = True
        if turn is not None:
            self.target_turn = self._clamp_axis(turn)
            changed = True
        if changed:
            self._print_drive_status()

    def _drive_active(self) -> bool:
        return abs(self.current_advance) > 1e-3 or abs(self.current_turn) > 1e-3

    def _update_drive_state(self) -> None:
        self.current_advance = self.target_advance
        self.current_turn = self.target_turn

    def _apply_external_payload(self, payload: dict[str, Any], sender: tuple[str, int]) -> None:
        paused = self._coerce_bool(payload, "paused")
        if paused is not None and paused != self.paused:
            self.paused = paused
            self._print_status("pausado" if self.paused else "reanudado")

        amplitude = self._coerce_float(payload, "amplitude")
        if amplitude is not None:
            self.gait.amplitude_scale = min(1.2, max(0.2, amplitude))
            self._print_status(f"amplitud={self.gait.amplitude_scale:.2f}")

        frequency = self._coerce_float(payload, "frequency")
        if frequency is not None:
            self.gait.frequency_hz = min(2.2, max(0.8, frequency))
            self._print_status(f"frecuencia={self.gait.frequency_hz:.2f} Hz")

        center = self._coerce_bool(payload, "center")
        if center:
            self.current_advance = 0.0
            self.current_turn = 0.0
            self._center_drive()

        reset = self._coerce_bool(payload, "reset")
        if reset:
            self.current_advance = 0.0
            self.current_turn = 0.0
            self.target_advance = 0.0
            self.target_turn = 0.0
            self.reset()
            self._print_status("simulacion reiniciada")

        advance = self._coerce_float(payload, "advance")
        turn = self._coerce_float(payload, "turn")
        if advance is not None or turn is not None:
            self._set_drive_targets(advance=advance, turn=turn)

        if payload:
            self._print_status(f"comando externo desde {sender[0]}:{sender[1]}")

    def _poll_external_control(self) -> None:
        if self.control_server is None:
            return
        for command in self.control_server.poll():
            try:
                self._apply_external_payload(command.payload, command.sender)
            except ValueError as exc:
                self._print_status(f"comando externo invalido: {exc}")

    def on_key(self, keycode: int) -> None:
        if keycode == 32:
            self._toggle_pause()
            return

        key = self._char_from_key(keycode)
        if key is None:
            return
        if key == "r":
            self.current_advance = 0.0
            self.current_turn = 0.0
            self._center_drive()
            self.reset()
            self._print_status("simulacion reiniciada")
        elif key == "w":
            self._adjust_advance(0.25)
        elif key == "s":
            self._adjust_advance(-0.25)
        elif key == "a":
            self._adjust_turn(0.25)
        elif key == "d":
            self._adjust_turn(-0.25)
        elif key == "x":
            self.current_advance = 0.0
            self.current_turn = 0.0
            self._center_drive()
        elif key == "j":
            self.gait.amplitude_scale = max(0.2, self.gait.amplitude_scale - 0.1)
            self._print_status(f"amplitud={self.gait.amplitude_scale:.2f}")
        elif key == "k":
            self.gait.amplitude_scale = min(1.2, self.gait.amplitude_scale + 0.1)
            self._print_status(f"amplitud={self.gait.amplitude_scale:.2f}")
        elif key == "n":
            self.gait.frequency_hz = max(0.8, self.gait.frequency_hz - 0.1)
            self._print_status(f"frecuencia={self.gait.frequency_hz:.2f} Hz")
        elif key == "m":
            self.gait.frequency_hz = min(2.2, self.gait.frequency_hz + 0.1)
            self._print_status(f"frecuencia={self.gait.frequency_hz:.2f} Hz")

    def _current_ctrl(self) -> list[tuple[int, float]]:
        gait_time = self.data.time
        phase = 2.0 * math.pi * self.gait.frequency_hz * gait_time
        left_phase = phase
        right_phase = phase + math.pi
        left_step = math.sin(left_phase)
        right_step = math.sin(right_phase)
        sway = math.cos(phase)
        advance = self.current_advance
        turn = self.current_turn
        advance_scale = abs(advance)
        scale = self.gait.amplitude_scale * (0.30 + 0.50 * advance_scale + 0.15 * abs(turn))
        left_swing = max(0.0, left_step)
        right_swing = max(0.0, right_step)
        left_stance = max(0.0, -left_step)
        right_stance = max(0.0, -right_step)
        stride = self.gait.hip_pitch * scale * (0.30 + 0.60 * advance_scale)
        hip_bias = 0.02 * advance
        knee_bias = 0.12 + 0.03 * advance_scale
        ankle_bias = -0.015 * advance
        waist_pitch = 0.02 * advance
        turn_roll_bias = 0.018 * turn
        hip_yaw_bias = 0.08 * turn
        waist_yaw_bias = -0.06 * turn
        left_height = float(self.data.site_xpos[self._foot_sites["left"]][2])
        right_height = float(self.data.site_xpos[self._foot_sites["right"]][2])
        target_left_height = self._neutral_foot_heights["left"] + self.gait.swing_height * scale * left_swing
        target_right_height = self._neutral_foot_heights["right"] + self.gait.swing_height * scale * right_swing
        left_lift_error = max(0.0, target_left_height - left_height)
        right_lift_error = max(0.0, target_right_height - right_height)
        left_knee = (
            knee_bias
            + self.gait.knee * scale * (0.95 * left_swing + 0.45 * left_stance)
            + 3.0 * left_lift_error
        )
        right_knee = (
            knee_bias
            + self.gait.knee * scale * (0.95 * right_swing + 0.45 * right_stance)
            + 3.0 * right_lift_error
        )
        left_ankle = (
            ankle_bias
            - self.gait.ankle_pitch * scale * left_swing
            + 0.10 * scale * left_stance
            + 1.0 * left_lift_error
        )
        right_ankle = (
            ankle_bias
            - self.gait.ankle_pitch * scale * right_swing
            + 0.10 * scale * right_stance
            + 1.0 * right_lift_error
        )
        left_roll = self.gait.hip_roll * scale * sway + turn_roll_bias
        right_roll = -self.gait.hip_roll * scale * sway - turn_roll_bias
        return [
            (
                self._actuators["left_hip_pitch_joint"],
                hip_bias + stride * advance * (left_swing - 0.35 * left_stance),
            ),
            (
                self._actuators["right_hip_pitch_joint"],
                hip_bias + stride * advance * (right_swing - 0.35 * right_stance),
            ),
            (self._actuators["left_knee_joint"], left_knee),
            (self._actuators["right_knee_joint"], right_knee),
            (self._actuators["left_ankle_pitch_joint"], left_ankle),
            (self._actuators["right_ankle_pitch_joint"], right_ankle),
            (self._actuators["left_hip_roll_joint"], left_roll),
            (self._actuators["right_hip_roll_joint"], right_roll),
            (self._actuators["left_ankle_roll_joint"], -0.55 * left_roll),
            (self._actuators["right_ankle_roll_joint"], -0.55 * right_roll),
            (self._actuators["left_hip_yaw_joint"], hip_yaw_bias),
            (self._actuators["right_hip_yaw_joint"], -hip_yaw_bias),
            (self._actuators["waist_pitch_joint"], waist_pitch),
            (self._actuators["waist_yaw_joint"], waist_yaw_bias),
            (self._actuators["left_shoulder_pitch_joint"], -self.gait.shoulder_pitch * scale * left_step),
            (self._actuators["right_shoulder_pitch_joint"], -self.gait.shoulder_pitch * scale * right_step),
        ]

    def _apply_walk_targets(self) -> None:
        ctrl = self._base_ctrl.copy()
        for actuator_id, delta in self._current_ctrl():
            ctrl[actuator_id] += delta
        self.data.ctrl[:] = ctrl

    def _clear_drive_assist(self) -> None:
        self.data.xfrc_applied[self._pelvis_id, :3] = 0.0

    def _apply_drive_assist(self) -> None:
        self._clear_drive_assist()
        if self._pelvis_height() < 0.5:
            return
        yaw = self._root_yaw()
        left_height = float(self.data.site_xpos[self._foot_sites["left"]][2])
        right_height = float(self.data.site_xpos[self._foot_sites["right"]][2])
        left_support = left_height <= self._neutral_foot_heights["left"] + 0.010
        right_support = right_height <= self._neutral_foot_heights["right"] + 0.010
        support_count = int(left_support) + int(right_support)
        if support_count == 0:
            return
        force = 55.0 * self.current_advance * self.gait.amplitude_scale * support_count
        self.data.xfrc_applied[self._pelvis_id, 0] = math.cos(yaw) * force
        self.data.xfrc_applied[self._pelvis_id, 1] = math.sin(yaw) * force

    def _root_yaw(self) -> float:
        quat_w, quat_x, quat_y, quat_z = self.data.qpos[3:7]
        return math.atan2(
            2.0 * (quat_w * quat_z + quat_x * quat_y),
            1.0 - 2.0 * (quat_y * quat_y + quat_z * quat_z),
        )

    def _pelvis_height(self) -> float:
        return float(self.data.xpos[self._pelvis_id][2])

    def _recover_if_fallen(self) -> None:
        if self._pelvis_height() >= 0.48:
            return
        self._print_status("caida detectada, reiniciando en stand")
        self.current_advance = 0.0
        self.current_turn = 0.0
        self.target_advance = 0.0
        self.target_turn = 0.0
        self.reset()

    def run(self, max_seconds: float | None = None) -> None:
        wall_start = time.perf_counter()
        with mujoco.viewer.launch_passive(
            self.model,
            self.data,
            key_callback=self.on_key if self.control_server is None else None,
            show_left_ui=False,
            show_right_ui=False,
        ) as viewer:
            if self.control_server is None:
                self._print_status(_HELP_TEXT)
            else:
                host, port = self.control_server.address
                self._print_status(
                    f"control externo UDP en {host}:{port} | usa send_unitree_command.py"
                )
            with viewer.lock():
                viewer.cam.distance = 3.0
                viewer.cam.elevation = -18
                viewer.cam.azimuth = 145
            viewer.sync()

            while viewer.is_running():
                step_start = time.perf_counter()
                self._poll_external_control()

                if not self.paused:
                    self._update_drive_state()
                    if self._drive_active():
                        self._apply_walk_targets()
                        self._apply_drive_assist()
                    else:
                        self.data.ctrl[:] = self._base_ctrl
                        self._clear_drive_assist()
                    mujoco.mj_step(self.model, self.data)
                    self._recover_if_fallen()

                viewer.sync()

                if max_seconds is not None and time.perf_counter() - wall_start >= max_seconds:
                    break

                remaining = self.model.opt.timestep - (time.perf_counter() - step_start)
                if remaining > 0:
                    time.sleep(remaining)


def launch_passive_viewer(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    keyframe: str | None,
    max_seconds: float | None = None,
    control_host: str = "127.0.0.1",
    control_port: int | None = None,
) -> None:
    if control_port is None:
        PassiveUnitreeSimulator(model, data, keyframe).run(max_seconds=max_seconds)
        return

    with UdpExternalControl(control_host, control_port) as control_server:
        host, port = control_server.address
        print(f"[control] escuchando UDP en {host}:{port}")
        PassiveUnitreeSimulator(
            model,
            data,
            keyframe,
            control_server=control_server,
        ).run(max_seconds=max_seconds)