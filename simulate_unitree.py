from __future__ import annotations

import argparse
from pathlib import Path
import sys

import mujoco

from interactive_unitree import launch_passive_viewer


ROOT = Path(__file__).resolve().parent
MENAGERIE_ROOT = ROOT / "third_party" / "mujoco_menagerie"
DEFAULT_KEYFRAME = "home"
ROBOT_SCENES = {
    "h1": MENAGERIE_ROOT / "unitree_h1" / "scene.xml",
    "g1": MENAGERIE_ROOT / "unitree_g1" / "scene.xml",
    "g1-hands": MENAGERIE_ROOT / "unitree_g1" / "scene_with_hands.xml",
}


def resolve_scene_path(robot: str, xml_path: str | None) -> Path:
    if xml_path is not None:
        scene_path = Path(xml_path).expanduser().resolve()
    else:
        scene_path = ROBOT_SCENES[robot]

    if not scene_path.exists():
        raise FileNotFoundError(
            f"No encontre el modelo en {scene_path}. "
            "Clona mujoco_menagerie en third_party/mujoco_menagerie "
            "o pasa --xml con una escena valida."
        )

    return scene_path


def compile_model(scene_path: Path) -> tuple[mujoco.MjModel, mujoco.MjData]:
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    return model, data


def apply_keyframe(model: mujoco.MjModel, data: mujoco.MjData, keyframe: str | None) -> None:
    if not keyframe:
        mujoco.mj_forward(model, data)
        return

    keyframe_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, keyframe)
    if keyframe_id < 0:
        if keyframe == DEFAULT_KEYFRAME:
            mujoco.mj_forward(model, data)
            return
        raise ValueError(f"El keyframe '{keyframe}' no existe en este modelo.")

    mujoco.mj_resetDataKeyframe(model, data, keyframe_id)
    mujoco.mj_forward(model, data)


def headless_smoke_test(model: mujoco.MjModel, data: mujoco.MjData, steps: int) -> None:
    for _ in range(steps):
        mujoco.mj_step(model, data)

    print(
        "MuJoCo listo:",
        f"nq={model.nq}",
        f"nv={model.nv}",
        f"nu={model.nu}",
        f"steps={steps}",
        f"time={data.time:.3f}",
    )


def launch_viewer(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    keyframe: str | None,
    max_seconds: float | None,
    control_host: str,
    control_port: int | None,
) -> None:
    launch_passive_viewer(
        model,
        data,
        keyframe=keyframe,
        max_seconds=max_seconds,
        control_host=control_host,
        control_port=control_port,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Lanza simulaciones MuJoCo con Unitree.")
    parser.add_argument("--robot", choices=sorted(ROBOT_SCENES), default="h1")
    parser.add_argument("--xml", help="Ruta opcional a una scene.xml personalizada.")
    parser.add_argument(
        "--keyframe",
        default=DEFAULT_KEYFRAME,
        help="Keyframe inicial a aplicar si existe. Usa --keyframe '' para desactivarlo.",
    )
    parser.add_argument(
        "--mode",
        choices=("headless", "viewer"),
        default="headless",
        help="headless compila y avanza la simulacion; viewer abre la ventana interactiva.",
    )
    parser.add_argument("--steps", type=int, default=300)
    parser.add_argument(
        "--max-seconds",
        type=float,
        help="Cierra el viewer automaticamente tras N segundos. Util para validar el arranque.",
    )
    parser.add_argument(
        "--control-host",
        default="127.0.0.1",
        help="Host local donde escuchar comandos UDP externos en modo viewer.",
    )
    parser.add_argument(
        "--control-port",
        type=int,
        default=47001,
        help="Puerto UDP para control externo. Usa 0 para desactivarlo.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    try:
        scene_path = resolve_scene_path(args.robot, args.xml)
        model, data = compile_model(scene_path)
        apply_keyframe(model, data, args.keyframe)
    except Exception as exc:
        print(f"Error preparando la simulacion: {exc}", file=sys.stderr)
        return 1

    if args.mode == "viewer":
        launch_viewer(
            model,
            data,
            args.keyframe,
            args.max_seconds,
            args.control_host,
            None if args.control_port == 0 else args.control_port,
        )
    else:
        headless_smoke_test(model, data, args.steps)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())