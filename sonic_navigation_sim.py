from __future__ import annotations

import argparse
import math
import pathlib
import time
import zlib

import msgpack
import mujoco
import numpy as np
import zmq

from gear_sonic.data.robot_model.instantiation.g1 import instantiate_g1_robot_model
from gear_sonic.utils.mujoco_sim import base_sim
from gear_sonic.utils.mujoco_sim.base_sim import BaseSimulator, DefaultEnv
from gear_sonic.utils.mujoco_sim.configs import SimLoopConfig
from gear_sonic.utils.mujoco_sim.simulator_factory import init_channel


class NavigationEnv(DefaultEnv):
    depth_bind = "tcp://*:5559"

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._depth_context = zmq.Context()
        self._depth_socket: zmq.Socket | None = None
        self._camera_id = mujoco.mj_name2id(
            self.mj_model, mujoco.mjtObj.mjOBJ_CAMERA, "head_camera"
        )
        if self._camera_id < 0:
            raise RuntimeError("La escena SONIC no contiene head_camera")

    def update_render_caches(self):
        render_caches = super().update_render_caches()
        if self._depth_socket is None:
            self._depth_socket = self._depth_context.socket(zmq.PUB)
            self._depth_socket.setsockopt(zmq.SNDHWM, 1)
            self._depth_socket.setsockopt(zmq.LINGER, 0)
            self._depth_socket.bind(self.depth_bind)
        renderer = self.renderers["ego_view"]
        renderer.enable_depth_rendering()
        renderer.update_scene(self.mj_data, camera="head_camera")
        depth = np.asarray(renderer.render(), dtype=np.float32)
        renderer.disable_depth_rendering()

        height, width = depth.shape
        fovy = float(self.mj_model.cam_fovy[self._camera_id])
        focal = 0.5 * height / math.tan(math.radians(fovy) * 0.5)
        message = {
            "topic": "sonic_depth",
            "timestamp": time.monotonic(),
            "shape": [height, width],
            "dtype": "float32",
            "depth_zlib": zlib.compress(depth.tobytes(), level=1),
            "fx": focal,
            "fy": focal,
            "cx": (width - 1) * 0.5,
            "cy": (height - 1) * 0.5,
            "camera_position": self.mj_data.cam_xpos[self._camera_id].tolist(),
            "camera_rotation": self.mj_data.cam_xmat[self._camera_id].reshape(3, 3).tolist(),
        }
        try:
            self._depth_socket.send(msgpack.packb(message, use_bin_type=True), flags=zmq.NOBLOCK)
        except zmq.Again:
            pass
        return render_caches

    def close_depth(self) -> None:
        if self._depth_socket is not None:
            self._depth_socket.close(linger=0)
        self._depth_context.term()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Simulador SONIC RGB-D para navegacion")
    parser.add_argument("--scene", required=True)
    parser.add_argument("--depth-bind", default="tcp://*:5559")
    parser.add_argument("--camera-port", type=int, default=5555)
    parser.add_argument("--onscreen", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    scene = pathlib.Path(args.scene).resolve()
    if not scene.is_file():
        raise FileNotFoundError(scene)

    config = SimLoopConfig(
        interface="sim",
        enable_onscreen=args.onscreen,
        enable_offscreen=True,
        enable_image_publish=True,
        camera_port=args.camera_port,
    )
    wbc_config = config.load_wbc_yaml()
    wbc_config["ENV_NAME"] = config.env_name
    wbc_config["ROBOT_SCENE"] = str(scene)
    NavigationEnv.depth_bind = args.depth_bind

    init_channel(config=wbc_config)
    base_sim.DefaultEnv = NavigationEnv
    robot_model = instantiate_g1_robot_model()
    simulator = BaseSimulator(
        config=wbc_config,
        env_name=config.env_name,
        onscreen=args.onscreen,
        offscreen=True,
        enable_image_publish=True,
    )
    simulator.start_image_publish_subprocess(config.mp_start_method, args.camera_port)
    try:
        print("SONIC_NAVIGATION_SIM_READY", flush=True)
        simulator.start()
    finally:
        simulator.sim_env.close_depth()


if __name__ == "__main__":
    main()