"""Detecta objetos por geometria (simulando el LiDAR/camara de profundidad del G1
real), no por un modelo de lenguaje. Idea: cualquier punto que sobresalga del piso
es un objeto candidato -- eso da la posicion 3D exacta sin adivinar nada. Despues,
el color (u opcionalmente un modelo de vision sobre un recorte chico) solo sirve
para ETIQUETAR cual objeto es cual, no para ubicarlo.

Este enfoque generaliza a cualquier objeto que sea un bulto geometrico sobre el
piso -- no esta hardcodeado a "una caja roja" como el primer intento con VLMs.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import cv2
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


@dataclass
class DetectedObject:
    pixel_centroid: tuple[float, float]
    world_position: np.ndarray
    pixel_count: int
    mean_rgb: tuple[int, int, int]


def render_rgb_and_depth(
    model: mujoco.MjModel, data: mujoco.MjData, cam_id: int, width: int, height: int
) -> tuple[np.ndarray, np.ndarray]:
    renderer = mujoco.Renderer(model, height=height, width=width)
    renderer.update_scene(data, camera=cam_id)
    rgb = renderer.render().copy()

    renderer.enable_depth_rendering()
    renderer.update_scene(data, camera=cam_id)
    depth = renderer.render().copy()
    renderer.disable_depth_rendering()

    return rgb, depth


def camera_intrinsics(model: mujoco.MjModel, cam_id: int, width: int, height: int) -> tuple[float, float, float, float]:
    fovy_deg = float(model.cam_fovy[cam_id])
    fy = height / (2.0 * np.tan(np.deg2rad(fovy_deg) / 2.0))
    fx = fy
    cx = width / 2.0
    cy = height / 2.0
    return fx, fy, cx, cy


def depth_to_point_cloud(
    depth: np.ndarray,
    cam_pos: np.ndarray,
    cam_mat: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
) -> np.ndarray:
    """Convierte cada pixel del mapa de profundidad en un punto 3D del mundo.
    Esto es, literalmente, lo que entrega un LiDAR/camara de profundidad real:
    una nube de puntos, no una lista de "objetos" -- la segmentacion es aparte.
    """
    height, width = depth.shape
    us, vs = np.meshgrid(np.arange(width), np.arange(height))
    x_cam = (us - cx) * depth / fx
    y_cam = -(vs - cy) * depth / fy
    z_cam = -depth
    points_cam = np.stack([x_cam, y_cam, z_cam], axis=-1)  # (H, W, 3)
    points_world = points_cam @ cam_mat.T + cam_pos
    return points_world


def segment_objects_above_floor(
    points_world: np.ndarray,
    rgb: np.ndarray,
    depth: np.ndarray,
    floor_height: float = 0.03,
    max_depth: float = 6.0,
    min_pixels: int = 80,
) -> list[DetectedObject]:
    height_map = points_world[:, :, 2]
    mask = ((height_map > floor_height) & (depth < max_depth) & (depth > 0.05)).astype(np.uint8)

    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)

    objects: list[DetectedObject] = []
    for label_id in range(1, num_labels):  # 0 es el fondo
        area = stats[label_id, cv2.CC_STAT_AREA]
        if area < min_pixels:
            continue

        blob_mask = labels == label_id
        world_points_blob = points_world[blob_mask]
        world_centroid = np.median(world_points_blob, axis=0)
        mean_rgb = rgb[blob_mask].mean(axis=0)
        u, v = centroids[label_id]

        objects.append(
            DetectedObject(
                pixel_centroid=(float(u), float(v)),
                world_position=world_centroid,
                pixel_count=int(area),
                mean_rgb=(int(mean_rgb[0]), int(mean_rgb[1]), int(mean_rgb[2])),
            )
        )

    return objects


def closest_color_name(rgb: tuple[int, int, int]) -> str:
    r, g, b = rgb
    reference = {
        "rojo": (211, 74, 54),
        "azul": (54, 115, 211),
        "gris/estante": (56, 56, 61),
        "marron/estante": (140, 105, 74),
    }
    best_name, best_dist = None, float("inf")
    for name, (rr, gg, bb) in reference.items():
        dist = (r - rr) ** 2 + (g - gg) ** 2 + (b - bb) ** 2
        if dist < best_dist:
            best_dist = dist
            best_name = name
    return best_name


def locate_objects(
    scene_path: Path,
    width: int,
    height: int,
    floor_height: float,
    save_debug_image: Path | None,
) -> list[DetectedObject]:
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "scene_cam")
    if cam_id < 0:
        raise ValueError("La escena no tiene una camara llamada 'scene_cam'.")

    rgb, depth = render_rgb_and_depth(model, data, cam_id, width, height)
    cam_pos = data.cam_xpos[cam_id].copy()
    cam_mat = data.cam_xmat[cam_id].reshape(3, 3).copy()
    fx, fy, cx, cy = camera_intrinsics(model, cam_id, width, height)

    points_world = depth_to_point_cloud(depth, cam_pos, cam_mat, fx, fy, cx, cy)
    objects = segment_objects_above_floor(points_world, rgb, depth, floor_height=floor_height)

    print(f"Objetos detectados por geometria (sin modelo de lenguaje): {len(objects)}")
    for i, obj in enumerate(objects):
        color_name = closest_color_name(obj.mean_rgb)
        print(
            f"  [{i}] pixel={obj.pixel_centroid} px_count={obj.pixel_count} "
            f"color~{color_name} rgb={obj.mean_rgb} "
            f"posicion_3d={obj.world_position}"
        )

    if save_debug_image is not None:
        debug = rgb.copy()
        for obj in objects:
            u, v = int(obj.pixel_centroid[0]), int(obj.pixel_centroid[1])
            cv2.drawMarker(debug, (u, v), (0, 255, 0), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
        Image.fromarray(debug).save(save_debug_image)
        print(f"Imagen de depuracion guardada en: {save_debug_image}")

    return objects


def compare_with_ground_truth(scene_path: Path, objects: list[DetectedObject], body_names: list[str]) -> None:
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    for body_name in body_names:
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if body_id < 0:
            print(f"Aviso: no encontre el body '{body_name}'.")
            continue
        ground_truth = data.xpos[body_id].copy()

        closest_obj = min(objects, key=lambda o: np.linalg.norm(o.world_position - ground_truth))
        error = float(np.linalg.norm(closest_obj.world_position - ground_truth))
        print(
            f"{body_name}: real={ground_truth}  detectado_mas_cercano={closest_obj.world_position}  "
            f"error={error * 100:.1f} cm"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Detecta objetos por geometria (LiDAR/depth simulado), sin depender de un modelo de vision."
    )
    parser.add_argument("--scene", default=str(DEFAULT_SCENE))
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--floor-height", type=float, default=0.03)
    parser.add_argument("--debug-image", default="/tmp/lidar_detections.png")
    parser.add_argument(
        "--ground-truth-bodies",
        nargs="*",
        default=["box_a", "box_b"],
        help="Bodies de la escena para comparar contra la posicion real.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    objects = locate_objects(
        Path(args.scene),
        args.width,
        args.height,
        args.floor_height,
        Path(args.debug_image) if args.debug_image else None,
    )
    if args.ground_truth_bodies:
        print()
        compare_with_ground_truth(Path(args.scene), objects, args.ground_truth_bodies)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
