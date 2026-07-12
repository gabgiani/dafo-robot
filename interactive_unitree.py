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
    "G marcha en el sitio | X centrado | R reinicia | I imprime sensores | J/K bajan/suben amplitud | N/M bajan/suben frecuencia"
)


@dataclass
class GaitTuning:
    # Pose base al caminar (crouch para bajar el centro de masa y dar margen de flexion)
    crouch_knee: float = 0.594
    crouch_hip: float = 0.26
    crouch_ankle: float = 0.30
    # Amplitudes de la zancada
    hip_stride: float = 0.55
    hip_push: float = 0.35
    knee_swing: float = 0.584
    ankle_swing: float = 0.32
    # Flexion de cadera al marchar en el sitio (levantar la pierna sin avanzar)
    march_hip: float = 0.17
    # Intensidad de la marcha en el sitio (mantiene el gesto suave)
    march_drive: float = 0.302
    # Transferencia lateral de peso (clave para que el pie de balanceo despegue sin resbalar)
    weight_shift: float = 0.137
    turn_roll: float = 0.05
    shoulder_pitch: float = 0.18
    # Ritmo
    frequency_hz: float = 0.977
    amplitude_scale: float = 0.6
    swing_height: float = 0.05
    command_alpha: float = 0.09
    # Rampa de activacion de la marcha (evita el salto brusco a la pose crouch)
    activation_alpha: float = 0.02
    # Seguimiento y estabilizacion por sensores
    forward_speed_scale: float = 0.45
    yaw_rate_scale: float = 0.65
    pitch_stabilizer: float = 0.35
    roll_stabilizer: float = 0.30
    yaw_stabilizer: float = 0.05
    height_stabilizer: float = 0.60
    # Asistencia de empuje en pelvis
    assist_command: float = 30.0
    assist_tracking: float = 45.0
    assist_limit: float = 70.0
    # Asistencia de balance (torque correctivo sobre la pelvis para mantener el torso erguido)
    balance_roll: float = 170.0
    balance_roll_d: float = 18.5
    balance_pitch: float = 52.5
    balance_pitch_d: float = 11.5
    balance_yaw_d: float = 10.0
    balance_lift: float = 339.0
    balance_torque_limit: float = 127.0


@dataclass
class DriveCommand:
    forward: float = 0.0
    lateral: float = 0.0
    turn: float = 0.0


@dataclass
class SensorSnapshot:
    phase: float
    pelvis_height: float
    pelvis_roll: float
    pelvis_pitch: float
    pelvis_yaw: float
    local_forward_velocity: float
    local_lateral_velocity: float
    yaw_rate: float
    left_foot_height: float
    right_foot_height: float
    left_target_height: float
    right_target_height: float
    left_support: bool
    right_support: bool
    imu_pelvis_gyro: tuple[float, float, float] | None
    imu_pelvis_accel: tuple[float, float, float] | None
    imu_torso_gyro: tuple[float, float, float] | None
    imu_torso_accel: tuple[float, float, float] | None


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
        self.target_lateral = 0.0
        self.target_turn = 0.0
        self.target_march = 0.0
        self.current_advance = 0.0
        self.current_lateral = 0.0
        self.current_turn = 0.0
        self.current_march = 0.0
        self._activation = 0.0
        self._keyframe_id = self._resolve_keyframe_id(keyframe)
        self._pelvis_id = self._body_id("pelvis")
        self._torso_site_id = self._site_id("imu_in_torso")
        self._base_ctrl = data.ctrl.copy()
        self._phase = 0.0
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
        self._sensor_slices = self._build_sensor_slices()
        self._neutral_foot_heights = self._capture_neutral_foot_heights()
        pelvis_xy = self.data.xpos[self._pelvis_id][:2]
        self._last_pelvis_xy = (float(pelvis_xy[0]), float(pelvis_xy[1]))
        self._neutral_pelvis_height = self._pelvis_height()

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

    def _build_sensor_slices(self) -> dict[str, slice]:
        sensor_slices: dict[str, slice] = {}
        for sensor_id in range(self.model.nsensor):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_id)
            if not name:
                continue
            start = int(self.model.sensor_adr[sensor_id])
            width = int(self.model.sensor_dim[sensor_id])
            sensor_slices[name] = slice(start, start + width)
        return sensor_slices

    def _read_named_sensor(self, name: str) -> tuple[float, ...] | None:
        sensor_slice = self._sensor_slices.get(name)
        if sensor_slice is None:
            return None
        values = self.data.sensordata[sensor_slice]
        return tuple(float(value) for value in values)

    def _quat_to_euler(self, quat_w: float, quat_x: float, quat_y: float, quat_z: float) -> tuple[float, float, float]:
        sinr_cosp = 2.0 * (quat_w * quat_x + quat_y * quat_z)
        cosr_cosp = 1.0 - 2.0 * (quat_x * quat_x + quat_y * quat_y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (quat_w * quat_y - quat_z * quat_x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (quat_w * quat_z + quat_x * quat_y)
        cosy_cosp = 1.0 - 2.0 * (quat_y * quat_y + quat_z * quat_z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    def _phase_values(self) -> tuple[float, float, float, float, float]:
        left_phase = self._phase
        right_phase = self._phase + math.pi
        left_step = math.sin(left_phase)
        right_step = math.sin(right_phase)
        sway = math.cos(self._phase)
        return left_phase, right_phase, left_step, right_step, sway

    def _advance_phase(self, dt: float) -> None:
        command_magnitude = max(
            abs(self.current_advance),
            abs(self.current_lateral),
            abs(self.current_turn),
            abs(self.current_march),
        )
        phase_rate = 2.0 * math.pi * self.gait.frequency_hz * (0.35 + 0.65 * command_magnitude)
        self._phase = (self._phase + phase_rate * dt) % (2.0 * math.pi)

    def _current_command(self) -> DriveCommand:
        return DriveCommand(
            forward=self.current_advance,
            lateral=self.current_lateral,
            turn=self.current_turn,
        )

    def _read_sensor_snapshot(self, dt: float, update_history: bool = True) -> SensorSnapshot:
        quat_w, quat_x, quat_y, quat_z = self.data.qpos[3:7]
        pelvis_roll, pelvis_pitch, pelvis_yaw = self._quat_to_euler(
            float(quat_w),
            float(quat_x),
            float(quat_y),
            float(quat_z),
        )

        pelvis_xy = self.data.xpos[self._pelvis_id][:2]
        current_xy = (float(pelvis_xy[0]), float(pelvis_xy[1]))
        if dt > 1e-9:
            world_vx = (current_xy[0] - self._last_pelvis_xy[0]) / dt
            world_vy = (current_xy[1] - self._last_pelvis_xy[1]) / dt
        else:
            world_vx = 0.0
            world_vy = 0.0
        if update_history:
            self._last_pelvis_xy = current_xy

        cos_yaw = math.cos(pelvis_yaw)
        sin_yaw = math.sin(pelvis_yaw)
        local_forward_velocity = cos_yaw * world_vx + sin_yaw * world_vy
        local_lateral_velocity = -sin_yaw * world_vx + cos_yaw * world_vy

        _, _, left_step, right_step, _ = self._phase_values()
        command = self._current_command()
        command_magnitude = max(abs(command.forward), abs(command.lateral), abs(command.turn))
        scale = self.gait.amplitude_scale * (0.25 + 0.65 * command_magnitude)
        left_swing = max(0.0, left_step)
        right_swing = max(0.0, right_step)

        left_foot_height = float(self.data.site_xpos[self._foot_sites["left"]][2])
        right_foot_height = float(self.data.site_xpos[self._foot_sites["right"]][2])
        left_target_height = self._neutral_foot_heights["left"] + self.gait.swing_height * scale * left_swing
        right_target_height = self._neutral_foot_heights["right"] + self.gait.swing_height * scale * right_swing
        support_margin = 0.012
        left_support = left_foot_height <= self._neutral_foot_heights["left"] + support_margin
        right_support = right_foot_height <= self._neutral_foot_heights["right"] + support_margin

        imu_pelvis_gyro = self._read_named_sensor("imu-pelvis-angular-velocity")
        imu_pelvis_accel = self._read_named_sensor("imu-pelvis-linear-acceleration")
        imu_torso_gyro = self._read_named_sensor("imu-torso-angular-velocity")
        imu_torso_accel = self._read_named_sensor("imu-torso-linear-acceleration")
        yaw_rate = imu_pelvis_gyro[2] if imu_pelvis_gyro is not None and len(imu_pelvis_gyro) >= 3 else 0.0

        return SensorSnapshot(
            phase=self._phase,
            pelvis_height=self._pelvis_height(),
            pelvis_roll=pelvis_roll,
            pelvis_pitch=pelvis_pitch,
            pelvis_yaw=pelvis_yaw,
            local_forward_velocity=local_forward_velocity,
            local_lateral_velocity=local_lateral_velocity,
            yaw_rate=float(yaw_rate),
            left_foot_height=left_foot_height,
            right_foot_height=right_foot_height,
            left_target_height=left_target_height,
            right_target_height=right_target_height,
            left_support=left_support,
            right_support=right_support,
            imu_pelvis_gyro=imu_pelvis_gyro,
            imu_pelvis_accel=imu_pelvis_accel,
            imu_torso_gyro=imu_torso_gyro,
            imu_torso_accel=imu_torso_accel,
        )

    def _format_sensor_snapshot(self, snapshot: SensorSnapshot) -> str:
        return (
            "sensores "
            f"h={snapshot.pelvis_height:.3f} "
            f"rpy=({snapshot.pelvis_roll:+.2f},{snapshot.pelvis_pitch:+.2f},{snapshot.pelvis_yaw:+.2f}) "
            f"v_local=({snapshot.local_forward_velocity:+.2f},{snapshot.local_lateral_velocity:+.2f}) "
            f"yaw_rate={snapshot.yaw_rate:+.2f} "
            f"pies=({snapshot.left_foot_height:.3f}/{snapshot.left_target_height:.3f},"
            f"{snapshot.right_foot_height:.3f}/{snapshot.right_target_height:.3f}) "
            f"support=({int(snapshot.left_support)},{int(snapshot.right_support)})"
        )

    def reset(self) -> None:
        if self._keyframe_id >= 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, self._keyframe_id)
        else:
            mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        self._base_ctrl = self.data.ctrl.copy()
        self._phase = 0.0
        self._activation = 0.0
        self._neutral_foot_heights = self._capture_neutral_foot_heights()
        pelvis_xy = self.data.xpos[self._pelvis_id][:2]
        self._last_pelvis_xy = (float(pelvis_xy[0]), float(pelvis_xy[1]))
        self._neutral_pelvis_height = self._pelvis_height()

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
            f"avance={self.target_advance:+.2f} lado={self.target_lateral:+.2f} giro={self.target_turn:+.2f}"
        )

    def _adjust_advance(self, delta: float) -> None:
        self.target_advance = self._clamp_axis(self.target_advance + delta)
        self._print_drive_status()

    def _adjust_turn(self, delta: float) -> None:
        self.target_turn = self._clamp_axis(self.target_turn + delta)
        self._print_drive_status()

    def _center_drive(self) -> None:
        self.target_advance = 0.0
        self.target_lateral = 0.0
        self.target_turn = 0.0
        self.target_march = 0.0
        self._print_status("joystick centrado")

    def _set_drive_targets(
        self,
        advance: float | None = None,
        lateral: float | None = None,
        turn: float | None = None,
    ) -> None:
        changed = False
        if advance is not None:
            self.target_advance = self._clamp_axis(advance)
            changed = True
        if lateral is not None:
            self.target_lateral = self._clamp_axis(lateral)
            changed = True
        if turn is not None:
            self.target_turn = self._clamp_axis(turn)
            changed = True
        if changed:
            self._print_drive_status()

    def _drive_active(self) -> bool:
        return (
            abs(self.current_advance) > 1e-3
            or abs(self.current_lateral) > 1e-3
            or abs(self.current_turn) > 1e-3
            or abs(self.current_march) > 1e-3
        )

    def _update_drive_state(self) -> None:
        alpha = self.gait.command_alpha
        self.current_advance += (self.target_advance - self.current_advance) * alpha
        self.current_lateral += (self.target_lateral - self.current_lateral) * alpha
        self.current_turn += (self.target_turn - self.current_turn) * alpha
        self.current_march += (self.target_march - self.current_march) * alpha
        active_target = 1.0 if (
            abs(self.target_advance) > 1e-3
            or abs(self.target_lateral) > 1e-3
            or abs(self.target_turn) > 1e-3
            or abs(self.target_march) > 1e-3
        ) else 0.0
        self._activation += (active_target - self._activation) * self.gait.activation_alpha

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
            self.current_lateral = 0.0
            self.current_turn = 0.0
            self.current_march = 0.0
            self._center_drive()

        reset = self._coerce_bool(payload, "reset")
        if reset:
            self.current_advance = 0.0
            self.current_lateral = 0.0
            self.current_turn = 0.0
            self.current_march = 0.0
            self.target_advance = 0.0
            self.target_lateral = 0.0
            self.target_turn = 0.0
            self.target_march = 0.0
            self.reset()
            self._print_status("simulacion reiniciada")

        march = self._coerce_float(payload, "march")
        if march is not None:
            self.target_march = max(0.0, min(1.0, march))
            self._print_status(f"marcha en el sitio={self.target_march:.2f}")

        advance = self._coerce_float(payload, "advance")
        lateral = self._coerce_float(payload, "lateral")
        if lateral is None:
            lateral = self._coerce_float(payload, "strafe")
        turn = self._coerce_float(payload, "turn")
        if advance is not None or lateral is not None or turn is not None:
            self._set_drive_targets(advance=advance, lateral=lateral, turn=turn)

        report_sensors = self._coerce_bool(payload, "report_sensors")
        if report_sensors:
            snapshot = self._read_sensor_snapshot(self.model.opt.timestep, update_history=False)
            self._print_status(self._format_sensor_snapshot(snapshot))

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
            self.current_lateral = 0.0
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
            self.current_lateral = 0.0
            self.current_turn = 0.0
            self._center_drive()
        elif key == "g":
            self.target_march = 0.0 if self.target_march > 0.5 else 1.0
            self._print_status(f"marcha en el sitio={self.target_march:.2f}")
        elif key == "i":
            snapshot = self._read_sensor_snapshot(self.model.opt.timestep, update_history=False)
            self._print_status(self._format_sensor_snapshot(snapshot))
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

    def _current_ctrl(self, snapshot: SensorSnapshot) -> list[tuple[int, float]]:
        command = self._current_command()
        _, _, left_step, right_step, _ = self._phase_values()
        g = self.gait
        fwd = command.forward
        turn = command.turn
        lat = command.lateral
        march = self.current_march
        # "step_drive" controla levantar la pierna y flexionar la rodilla (marcha o avance).
        # El empuje hacia adelante depende solo de "fwd".
        step_drive = max(abs(fwd), 0.6 * abs(turn), abs(lat), g.march_drive * march)

        # Envolvente de balanceo/apoyo por pierna en [0, 1].
        left_swing = max(0.0, left_step)
        right_swing = max(0.0, right_step)
        left_stance = max(0.0, -left_step)
        right_stance = max(0.0, -right_step)

        # Errores de realimentacion tomados de sensores reales.
        forward_error = g.forward_speed_scale * fwd - snapshot.local_forward_velocity
        yaw_error = g.yaw_rate_scale * turn - snapshot.yaw_rate
        height_error = self._neutral_pelvis_height - snapshot.pelvis_height
        pitch_correction = g.pitch_stabilizer * (-snapshot.pelvis_pitch)
        roll_correction = g.roll_stabilizer * (-snapshot.pelvis_roll)
        yaw_correction = g.yaw_stabilizer * yaw_error
        knee_hold = g.height_stabilizer * max(0.0, height_error)

        # Pose base agachada: solo se agacha cuando hay comando (marcha o avance).
        crouch = 0.35 + 0.65 * step_drive
        base_knee = g.crouch_knee * crouch + knee_hold
        base_hip = g.crouch_hip * crouch
        base_ankle = -g.crouch_ankle * crouch

        # Transferencia lateral de peso hacia la pierna de apoyo: descarga el pie de balanceo.
        shift = g.weight_shift * step_drive * left_step

        # Cadera (pitch): flexion para levantar la pierna (marcha + avance) y empuje de la
        # pierna de apoyo hacia atras (solo con avance).
        hip_lift = g.hip_stride * fwd + g.march_hip * march
        left_hip_pitch = (
            base_hip
            + hip_lift * left_swing
            - g.hip_push * fwd * left_stance
            + pitch_correction
        )
        right_hip_pitch = (
            base_hip
            + hip_lift * right_swing
            - g.hip_push * fwd * right_stance
            + pitch_correction
        )

        # Rodilla: crouch + flexion extra en balanceo para levantar y librar el pie.
        left_knee = base_knee + g.knee_swing * step_drive * left_swing
        right_knee = base_knee + g.knee_swing * step_drive * right_swing

        # Tobillo: mantiene el pie plano y dorsiflexiona en balanceo.
        left_ankle = base_ankle + g.ankle_swing * step_drive * left_swing
        right_ankle = base_ankle + g.ankle_swing * step_drive * right_swing

        # Roll de cadera: transferencia de peso + correccion de sensor + giro.
        left_roll = shift + roll_correction + g.turn_roll * turn
        right_roll = shift + roll_correction - g.turn_roll * turn

        return [
            (self._actuators["left_hip_pitch_joint"], left_hip_pitch),
            (self._actuators["right_hip_pitch_joint"], right_hip_pitch),
            (self._actuators["left_knee_joint"], left_knee),
            (self._actuators["right_knee_joint"], right_knee),
            (self._actuators["left_ankle_pitch_joint"], left_ankle),
            (self._actuators["right_ankle_pitch_joint"], right_ankle),
            (self._actuators["left_hip_roll_joint"], left_roll),
            (self._actuators["right_hip_roll_joint"], right_roll),
            (self._actuators["left_ankle_roll_joint"], -0.5 * left_roll),
            (self._actuators["right_ankle_roll_joint"], -0.5 * right_roll),
            (self._actuators["left_hip_yaw_joint"], 0.05 * turn + yaw_correction),
            (self._actuators["right_hip_yaw_joint"], -0.05 * turn + yaw_correction),
            (self._actuators["waist_pitch_joint"], 0.12 * height_error + 0.3 * pitch_correction),
            (self._actuators["waist_yaw_joint"], -0.04 * turn),
            (self._actuators["left_shoulder_pitch_joint"], -g.shoulder_pitch * fwd * left_step),
            (self._actuators["right_shoulder_pitch_joint"], -g.shoulder_pitch * fwd * right_step),
        ]

    def _apply_walk_targets(self, snapshot: SensorSnapshot) -> None:
        ctrl = self._base_ctrl.copy()
        blend = self._activation
        for actuator_id, delta in self._current_ctrl(snapshot):
            ctrl[actuator_id] += blend * delta
        self.data.ctrl[:] = ctrl

    def _clear_drive_assist(self) -> None:
        self.data.xfrc_applied[self._pelvis_id, :6] = 0.0

    def _apply_balance_assist(self, snapshot: SensorSnapshot) -> None:
        # Torque correctivo sobre la pelvis para mantener el torso erguido y a altura,
        # dejando que las piernas hagan el paso. Usa orientacion + giroscopio (IMU).
        g = self.gait
        gyro = snapshot.imu_pelvis_gyro or (0.0, 0.0, 0.0)
        roll_torque = -g.balance_roll * snapshot.pelvis_roll - g.balance_roll_d * gyro[0]
        pitch_torque = -g.balance_pitch * snapshot.pelvis_pitch - g.balance_pitch_d * gyro[1]
        yaw_torque = -g.balance_yaw_d * gyro[2]
        limit = g.balance_torque_limit
        self.data.xfrc_applied[self._pelvis_id, 3] = max(-limit, min(limit, roll_torque))
        self.data.xfrc_applied[self._pelvis_id, 4] = max(-limit, min(limit, pitch_torque))
        self.data.xfrc_applied[self._pelvis_id, 5] = max(-limit, min(limit, yaw_torque))
        height_error = self._neutral_pelvis_height - snapshot.pelvis_height
        self.data.xfrc_applied[self._pelvis_id, 2] = g.balance_lift * height_error

    def _apply_drive_assist(self, snapshot: SensorSnapshot) -> None:
        self._clear_drive_assist()
        if self._pelvis_height() < 0.45:
            return
        self._apply_balance_assist(snapshot)
        support_count = int(snapshot.left_support) + int(snapshot.right_support)
        if support_count == 0:
            return
        yaw = snapshot.pelvis_yaw
        target_forward_velocity = self.gait.forward_speed_scale * self.current_advance
        velocity_error = target_forward_velocity - snapshot.local_forward_velocity
        command_force = self.gait.assist_command * self.current_advance * self.gait.amplitude_scale * support_count
        tracking_force = self.gait.assist_tracking * velocity_error * support_count
        limit = self.gait.assist_limit
        force = max(-limit, min(limit, command_force + tracking_force))
        self.data.xfrc_applied[self._pelvis_id, 0] += math.cos(yaw) * force
        self.data.xfrc_applied[self._pelvis_id, 1] += math.sin(yaw) * force

    def _root_yaw(self) -> float:
        quat_w, quat_x, quat_y, quat_z = self.data.qpos[3:7]
        return math.atan2(
            2.0 * (quat_w * quat_z + quat_x * quat_y),
            1.0 - 2.0 * (quat_y * quat_y + quat_z * quat_z),
        )

    def _pelvis_height(self) -> float:
        return float(self.data.xpos[self._pelvis_id][2])

    def _configure_initial_camera(self, viewer: Any) -> None:
        torso_pos = self.data.site_xpos[self._torso_site_id]
        viewer.cam.lookat[0] = float(torso_pos[0])
        viewer.cam.lookat[1] = float(torso_pos[1])
        viewer.cam.lookat[2] = float(torso_pos[2]) - 0.02
        viewer.cam.distance = 3.35
        viewer.cam.elevation = -14
        viewer.cam.azimuth = 145

    def _recover_if_fallen(self) -> None:
        if self._pelvis_height() >= 0.42:
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
                self._configure_initial_camera(viewer)
            viewer.sync()

            while viewer.is_running():
                step_start = time.perf_counter()
                self._poll_external_control()

                if not self.paused:
                    self._update_drive_state()
                    if self._drive_active():
                        self._advance_phase(self.model.opt.timestep)
                        snapshot = self._read_sensor_snapshot(self.model.opt.timestep)
                        self._apply_walk_targets(snapshot)
                        self._apply_drive_assist(snapshot)
                    else:
                        self.data.ctrl[:] = self._base_ctrl
                        self._clear_drive_assist()
                        self._read_sensor_snapshot(self.model.opt.timestep)
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