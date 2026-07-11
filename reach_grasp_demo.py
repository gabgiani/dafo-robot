from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import shutil
import subprocess
import sys

import mujoco
import numpy as np

from simulate_unitree import compile_model, apply_keyframe


ROOT = Path(__file__).resolve().parent
SCENE_PATH = ROOT / "scenes" / "g1_reach_grasp_scene.xml"
KEYFRAME_NAME = "stand"
DEFAULT_VIDEO_PATH = ROOT / "artifacts" / "g1_reach_grasp.mp4"

RIGHT_ARM_REACH = {
    "right_shoulder_pitch_joint": -0.2,
    "right_shoulder_roll_joint": -0.9,
    "right_shoulder_yaw_joint": 0.0,
    "right_elbow_joint": 1.0,
    "right_wrist_roll_joint": 0.0,
    "right_wrist_pitch_joint": 0.0,
    "right_wrist_yaw_joint": 0.0,
}

RIGHT_HAND_CLOSE = {
    "right_hand_thumb_0_joint": 0.6,
    "right_hand_thumb_1_joint": -0.35,
    "right_hand_thumb_2_joint": -0.7,
    "right_hand_index_0_joint": 0.9,
    "right_hand_index_1_joint": 1.1,
    "right_hand_middle_0_joint": 0.9,
    "right_hand_middle_1_joint": 1.1,
}


@dataclass(frozen=True)
class Phase:
    name: str
    duration: float
    targets: dict[str, float]


PHASES = (
    Phase("settle", 0.6, {}),
    Phase("reach", 1.4, RIGHT_ARM_REACH),
    Phase("close", 0.8, RIGHT_ARM_REACH | RIGHT_HAND_CLOSE),
    Phase("hold", 0.6, RIGHT_ARM_REACH | RIGHT_HAND_CLOSE),
    Phase("reopen", 0.8, RIGHT_ARM_REACH),
)


class VideoWriter:
    def __init__(self, output_path: Path, width: int, height: int, fps: int):
        ffmpeg = shutil.which("ffmpeg")
        if ffmpeg is None:
            raise RuntimeError("No encontre ffmpeg para exportar el video.")

        output_path.parent.mkdir(parents=True, exist_ok=True)
        command = [
            ffmpeg,
            "-y",
            "-f",
            "rawvideo",
            "-pixel_format",
            "rgb24",
            "-video_size",
            f"{width}x{height}",
            "-framerate",
            str(fps),
            "-i",
            "-",
            "-an",
            "-vcodec",
            "libx264",
            "-pix_fmt",
            "yuv420p",
            str(output_path),
        ]
        self._process = subprocess.Popen(command, stdin=subprocess.PIPE)

    def write(self, frame: np.ndarray) -> None:
        if self._process.stdin is None:
            raise RuntimeError("ffmpeg no acepto el stream de video.")
        self._process.stdin.write(frame.tobytes())

    def close(self) -> None:
        if self._process.stdin is not None:
            self._process.stdin.close()
        exit_code = self._process.wait()
        if exit_code != 0:
            raise RuntimeError(f"ffmpeg termino con codigo {exit_code}.")


def actuator_id(model: mujoco.MjModel, name: str) -> int:
    actuator = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    if actuator < 0:
      raise ValueError(f"No encontre el actuador '{name}'.")
    return actuator


def body_position(model: mujoco.MjModel, data: mujoco.MjData, body_name: str) -> np.ndarray:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if body_id < 0:
        raise ValueError(f"No encontre el body '{body_name}'.")
    return data.xpos[body_id].copy()


def site_position(model: mujoco.MjModel, data: mujoco.MjData, site_name: str) -> np.ndarray:
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
    if site_id < 0:
        raise ValueError(f"No encontre el site '{site_name}'.")
    return data.site_xpos[site_id].copy()


def target_ctrl(ctrl: np.ndarray, model: mujoco.MjModel, overrides: dict[str, float]) -> np.ndarray:
    next_ctrl = ctrl.copy()
    for name, value in overrides.items():
        next_ctrl[actuator_id(model, name)] = value
    return next_ctrl


def pinch_center(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    thumb = body_position(model, data, "right_hand_thumb_2_link")
    index_finger = body_position(model, data, "right_hand_index_1_link")
    middle_finger = body_position(model, data, "right_hand_middle_1_link")
    return (thumb + index_finger + middle_finger) / 3.0


def run_demo(model: mujoco.MjModel, data: mujoco.MjData, fps: int, width: int, height: int, output: Path | None) -> tuple[float, float]:
    renderer = None
    writer = None
    steps_per_frame = max(1, round(1.0 / (fps * model.opt.timestep)))
    min_distance = float("inf")
    total_steps = 0

    try:
        if output is not None:
            renderer = mujoco.Renderer(model, height=height, width=width)
            writer = VideoWriter(output, width, height, fps)

        for phase in PHASES:
            start_ctrl = data.ctrl.copy()
            end_ctrl = target_ctrl(start_ctrl, model, phase.targets)
            phase_steps = max(1, round(phase.duration / model.opt.timestep))

            for step in range(phase_steps):
                alpha = (step + 1) / phase_steps
                data.ctrl[:] = start_ctrl + alpha * (end_ctrl - start_ctrl)
                mujoco.mj_step(model, data)
                total_steps += 1

                target = site_position(model, data, "grasp_target")
                distance = np.linalg.norm(pinch_center(model, data) - target)
                min_distance = min(min_distance, float(distance))

                if renderer is not None and writer is not None and total_steps % steps_per_frame == 0:
                    renderer.update_scene(data)
                    writer.write(renderer.render())

        return data.time, min_distance
    finally:
        if writer is not None:
            writer.close()
        if renderer is not None:
            renderer.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Demo simple de reach-and-grasp para Unitree G1 con manos.")
    parser.add_argument("--output", type=Path, default=DEFAULT_VIDEO_PATH)
    parser.add_argument("--no-video", action="store_true")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=720)
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    try:
        model, data = compile_model(SCENE_PATH)
        apply_keyframe(model, data, KEYFRAME_NAME)
        output = None if args.no_video else args.output
        sim_time, min_distance = run_demo(model, data, args.fps, args.width, args.height, output)
    except Exception as exc:
        print(f"Error ejecutando la demo: {exc}", file=sys.stderr)
        return 1

    if output is None:
        print(
            "Demo reach-grasp lista:",
            f"time={sim_time:.2f}",
            f"min_grasp_distance={min_distance:.3f}",
        )
    else:
        print(
            "Demo reach-grasp exportada:",
            f"video={output}",
            f"time={sim_time:.2f}",
            f"min_grasp_distance={min_distance:.3f}",
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())