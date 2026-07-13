"""Localiza objetos en 3D combinando deteccion por lenguaje (minicpm-v via Ollama)
con el mapa de profundidad de la camara de la escena (equivalente en simulacion
a la camara de profundidad + LiDAR 3D que trae de fabrica el Unitree G1).

Flujo:
1. Renderiza RGB y profundidad desde la camara fija de la escena.
2. Le pide a un modelo de vision (minicpm-v, corriendo en Ollama) que ubique
   un objeto descrito en texto (no hardcodeado a un color) -> bounding box 2D.
3. Cruza el centro de esa caja con el mapa de profundidad -> punto 3D exacto,
   usando la geometria real de la camara (posicion, orientacion, fov).
4. Si se pasa --ground-truth-body, compara contra la posicion real conocida
   por la simulacion para validar el error de localizacion.
"""
from __future__ import annotations

import argparse
import base64
import json
import re
import urllib.request
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parent
DEFAULT_SCENE = (
    ROOT
    / "third_party"
    / "unitree_rl_gym"
    / "resources"
    / "robots"
    / "g1_description"
    / "g1_warehouse_scene.xml"
)


def render_rgb_and_depth(
    model: mujoco.MjModel, data: mujoco.MjData, cam_id: int, width: int, height: int
) -> tuple[np.ndarray, np.ndarray]:
    renderer = mujoco.Renderer(model, height=height, width=width)
    renderer.update_scene(data, camera=cam_id)
    rgb = renderer.render()

    renderer.enable_depth_rendering()
    renderer.update_scene(data, camera=cam_id)
    depth = renderer.render().copy()
    renderer.disable_depth_rendering()

    return rgb, depth


def ask_vision_model(
    host: str, model: str, rgb: np.ndarray, query: str
) -> tuple[float, float, float, float]:
    buf_path = Path("/tmp/_locate_object_frame.png")
    Image.fromarray(rgb).save(buf_path)
    image_b64 = base64.b64encode(buf_path.read_bytes()).decode("utf-8")

    payload = {
        "model": model,
        "prompt": (
            f"Detect the {query}. "
            'Respond ONLY with a JSON bounding box like {"x_min":.., "y_min":.., '
            '"x_max":.., "y_max":..}.'
        ),
        "images": [image_b64],
        "stream": False,
    }
    req = urllib.request.Request(
        f"http://{host}:11434/api/generate",
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
    )
    with urllib.request.urlopen(req, timeout=60) as resp:
        result = json.loads(resp.read().decode("utf-8"))

    text = result.get("response", "")
    numbers = re.findall(r"-?\d*\.?\d+", text)
    if len(numbers) < 4:
        raise ValueError(f"El modelo no devolvio 4 numeros. Respuesta cruda: {text!r}")

    x_min, y_min, x_max, y_max = (float(n) for n in numbers[:4])
    return x_min, y_min, x_max, y_max


def camera_intrinsics(model: mujoco.MjModel, cam_id: int, width: int, height: int) -> tuple[float, float, float, float]:
    fovy_deg = float(model.cam_fovy[cam_id])
    fy = height / (2.0 * np.tan(np.deg2rad(fovy_deg) / 2.0))
    fx = fy  # MuJoCo asume pixeles cuadrados con este modelo de camara.
    cx = width / 2.0
    cy = height / 2.0
    return fx, fy, cx, cy


def pixel_depth_to_world(
    u: float,
    v: float,
    depth: float,
    cam_pos: np.ndarray,
    cam_mat: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
) -> np.ndarray:
    # Convencion de camara de MuJoCo: mira hacia -Z local, +Y local es "arriba".
    x_cam = (u - cx) * depth / fx
    y_cam = -(v - cy) * depth / fy
    z_cam = -depth
    point_cam = np.array([x_cam, y_cam, z_cam])
    return cam_mat @ point_cam + cam_pos


def locate_object(
    scene_path: Path,
    query: str,
    host: str,
    model_name: str,
    width: int,
    height: int,
    ground_truth_body: str | None,
) -> np.ndarray:
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "scene_cam")
    if cam_id < 0:
        raise ValueError("La escena no tiene una camara llamada 'scene_cam'.")

    rgb, depth = render_rgb_and_depth(model, data, cam_id, width, height)
    x_min, y_min, x_max, y_max = ask_vision_model(host, model_name, rgb, query)

    u = (x_min + x_max) / 2.0 * width
    v = (y_min + y_max) / 2.0 * height
    u_i, v_i = int(np.clip(u, 0, width - 1)), int(np.clip(v, 0, height - 1))
    depth_at_pixel = float(depth[v_i, u_i])

    cam_pos = data.cam_xpos[cam_id].copy()
    cam_mat = data.cam_xmat[cam_id].reshape(3, 3).copy()
    fx, fy, cx, cy = camera_intrinsics(model, cam_id, width, height)

    world_point = pixel_depth_to_world(u, v, depth_at_pixel, cam_pos, cam_mat, fx, fy, cx, cy)

    print(f"Consulta: {query!r}")
    print(f"Bounding box normalizado: ({x_min:.3f}, {y_min:.3f}, {x_max:.3f}, {y_max:.3f})")
    print(f"Pixel central: ({u:.1f}, {v:.1f})  profundidad: {depth_at_pixel:.3f} m")
    print(f"Posicion 3D calculada (mundo): {world_point}")

    if ground_truth_body:
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, ground_truth_body)
        if body_id < 0:
            print(f"Aviso: no encontre el body '{ground_truth_body}' para comparar.")
        else:
            ground_truth = data.xpos[body_id].copy()
            error = float(np.linalg.norm(world_point - ground_truth))
            print(f"Posicion real ({ground_truth_body}): {ground_truth}")
            print(f"Error de localizacion: {error * 100:.1f} cm")

    return world_point


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Localiza en 3D un objeto descrito en texto, combinando deteccion por lenguaje + profundidad."
    )
    parser.add_argument("--scene", default=str(DEFAULT_SCENE))
    parser.add_argument("--query", default="the blue box", help="Descripcion en texto del objeto a ubicar.")
    parser.add_argument("--host", default="10.8.0.3", help="Host de Ollama (192.168.0.60 o 10.8.0.3).")
    parser.add_argument("--model", default="moondream")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument(
        "--ground-truth-body",
        help="Nombre de un body de la escena para comparar contra la posicion real (ej: box_b).",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    locate_object(
        Path(args.scene),
        args.query,
        args.host,
        args.model,
        args.width,
        args.height,
        args.ground_truth_body,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
