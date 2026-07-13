"""Prueba descartable: renderiza el escenario del almacen y le pregunta a Ollama (gemma4:e2b) por las cajas."""
from __future__ import annotations

import base64
import json
import urllib.request
from pathlib import Path

import mujoco

ROOT = Path(__file__).resolve().parent
SCENE = (
    ROOT
    / "third_party"
    / "unitree_rl_gym"
    / "resources"
    / "robots"
    / "g1_description"
    / "g1_warehouse_scene.xml"
)
OUT_IMG = ROOT / "artifacts" / "_tmp_warehouse_render.png"
OLLAMA_HOST = "192.168.0.60"
MODEL = "gemma4:e2b"


def render_scene() -> bytes:
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    renderer = mujoco.Renderer(model, height=480, width=640)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [2.5, 0, 0.4]
    cam.distance = 4.0
    cam.azimuth = -140
    cam.elevation = -20
    renderer.update_scene(data, camera=cam)
    pixels = renderer.render()

    import numpy as np
    from PIL import Image

    OUT_IMG.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(pixels).save(OUT_IMG)
    return OUT_IMG.read_bytes()


def ask_ollama(image_bytes: bytes) -> str:
    payload = {
        "model": MODEL,
        "prompt": "Describe the scene. List every distinct colored box you can see and its approximate position (left/center/right).",
        "images": [base64.b64encode(image_bytes).decode("utf-8")],
        "stream": False,
    }
    req = urllib.request.Request(
        f"http://{OLLAMA_HOST}:11434/api/generate",
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
    )
    with urllib.request.urlopen(req, timeout=120) as resp:
        result = json.loads(resp.read().decode("utf-8"))
    return result.get("response", "")


def main() -> None:
    image_bytes = render_scene()
    print(f"Imagen renderizada: {OUT_IMG} ({len(image_bytes)} bytes)")
    answer = ask_ollama(image_bytes)
    print("--- respuesta de", MODEL, "---")
    print(answer)


if __name__ == "__main__":
    main()
