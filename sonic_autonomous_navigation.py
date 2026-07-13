from __future__ import annotations

import argparse
import math
import socket
import time
from urllib.parse import urlparse

import numpy as np
import zmq

from sonic_navigation_planner import (
    DepthFrame,
    DepthSubscriber,
    astar,
    build_occupancy_grid,
    depth_to_world_points,
    select_waypoint,
)
from sonic_remote_control import IDLE, command_message, planner_message
from sonic_telemetry import SonicPhysicalPose, SonicPhysicalPoseSubscriber, SonicTelemetrySubscriber


WALK = 2
PUBLISH_PERIOD = 0.05


def yaw_from_pose(pose: SonicPhysicalPose) -> float:
    qw, qx, qy, qz = pose.quaternion
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def angle_difference(target: float, current: float) -> float:
    return math.atan2(math.sin(target - current), math.cos(target - current))


def wait_for_endpoint(url: str, timeout: float) -> None:
    parsed = urlparse(url)
    if parsed.scheme != "tcp" or parsed.hostname is None or parsed.port is None:
        raise ValueError(f"Endpoint TCP no valido: {url}")
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            with socket.create_connection((parsed.hostname, parsed.port), timeout=0.2):
                return
        except OSError:
            time.sleep(0.05)
    raise TimeoutError(f"SONIC no escucha en {url}")


def wait_for_inputs(
    pose_subscriber: SonicPhysicalPoseSubscriber,
    depth_subscriber: DepthSubscriber,
    timeout: float,
) -> tuple[SonicPhysicalPose, DepthFrame]:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for subscriber in (pose_subscriber, depth_subscriber):
            error = subscriber.error()
            if error is not None:
                raise error
        pose = pose_subscriber.latest()
        depth = depth_subscriber.latest()
        if pose is not None and depth is not None:
            return pose, depth
        time.sleep(0.02)
    raise TimeoutError("No se recibieron pose fisica y depth antes del timeout")


def publish_for(publisher: zmq.Socket, message: bytes, duration: float) -> None:
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        publisher.send(message)
        time.sleep(PUBLISH_PERIOD)


def require_fresh(name: str, received_at: float, stale_timeout: float) -> None:
    age = time.monotonic() - received_at
    if age > stale_timeout:
        raise RuntimeError(f"{name} stale: {age:.2f} s")


def main() -> None:
    parser = argparse.ArgumentParser(description="Navegacion autonoma A* para SONIC")
    parser.add_argument("--bind", default="tcp://127.0.0.1:5556")
    parser.add_argument("--telemetry-url", default="tcp://127.0.0.1:5557")
    parser.add_argument("--pose-url", default="tcp://127.0.0.1:5558")
    parser.add_argument("--depth-url", default="tcp://127.0.0.1:5559")
    parser.add_argument("--goal-x", type=float, default=4.5)
    parser.add_argument("--goal-y", type=float, default=0.0)
    parser.add_argument("--speed", type=float, default=0.25)
    parser.add_argument("--resolution", type=float, default=0.1)
    parser.add_argument("--inflation-radius", type=float, default=0.45)
    parser.add_argument("--self-filter-radius", type=float, default=0.35)
    parser.add_argument("--critical-clearance", type=float, default=0.3)
    parser.add_argument("--goal-tolerance", type=float, default=0.35)
    parser.add_argument("--lookahead", type=float, default=0.6)
    parser.add_argument("--max-depth", type=float, default=5.0)
    parser.add_argument("--timeout", type=float, default=90.0)
    parser.add_argument("--startup-timeout", type=float, default=20.0)
    parser.add_argument("--stale-timeout", type=float, default=0.6)
    parser.add_argument("--fall-height", type=float, default=0.45)
    args = parser.parse_args()
    if not 0.1 <= args.speed <= 0.5:
        parser.error("--speed debe estar entre 0.1 y 0.5 m/s")
    if args.critical_clearance >= args.inflation_radius:
        parser.error("--critical-clearance debe ser menor que --inflation-radius")
    if args.self_filter_radius >= args.inflation_radius:
        parser.error("--self-filter-radius debe ser menor que --inflation-radius")

    context = zmq.Context()
    publisher = context.socket(zmq.PUB)
    publisher.setsockopt(zmq.LINGER, 0)
    publisher.bind(args.bind)
    pose_subscriber = SonicPhysicalPoseSubscriber(args.pose_url)
    telemetry_subscriber = SonicTelemetrySubscriber(args.telemetry_url)
    depth_subscriber = DepthSubscriber(args.depth_url)
    pose_subscriber.start()
    telemetry_subscriber.start()
    depth_subscriber.start()
    safe_idle: bytes | None = None
    activated = False
    goal = np.array([args.goal_x, args.goal_y], dtype=float)

    try:
        time.sleep(0.6)
        wait_for_endpoint(args.telemetry_url, args.startup_timeout)
        pose, _ = wait_for_inputs(pose_subscriber, depth_subscriber, args.startup_timeout)
        commanded_yaw = yaw_from_pose(pose)
        initial_facing = (math.cos(commanded_yaw), math.sin(commanded_yaw), 0.0)
        safe_idle = planner_message(IDLE, (0.0, 0.0, 0.0), initial_facing, -1.0)
        activated = True
        activation_deadline = time.monotonic() + 2.0
        while time.monotonic() < activation_deadline:
            publisher.send(command_message(start=True, stop=False))
            publisher.send(safe_idle)
            time.sleep(PUBLISH_PERIOD)

        telemetry_deadline = time.monotonic() + args.startup_timeout
        while telemetry_subscriber.latest() is None and time.monotonic() < telemetry_deadline:
            error = telemetry_subscriber.error()
            if error is not None:
                raise error
            time.sleep(0.02)
        if telemetry_subscriber.latest() is None:
            raise TimeoutError("SONIC no publico telemetria despues de activar")

        started_at = time.monotonic()
        last_update = started_at
        last_report = 0.0
        waypoint: np.ndarray | None = None
        path_cells = 0
        while True:
            now = time.monotonic()
            if now - started_at > args.timeout:
                raise TimeoutError("La navegacion excedio el timeout")
            pose = pose_subscriber.latest()
            depth = depth_subscriber.latest()
            telemetry = telemetry_subscriber.latest()
            if pose is None or depth is None or telemetry is None:
                raise RuntimeError("Se perdio una fuente de telemetria")
            for name, subscriber in (
                ("pose", pose_subscriber),
                ("depth", depth_subscriber),
                ("telemetria SONIC", telemetry_subscriber),
            ):
                error = subscriber.error()
                if error is not None:
                    raise error
            require_fresh("pose", pose.received_at, args.stale_timeout)
            require_fresh("depth", depth.received_at, args.stale_timeout)
            require_fresh("telemetria SONIC", telemetry.received_at, args.stale_timeout)
            if pose.position[2] < args.fall_height:
                raise RuntimeError(f"Caida detectada: altura {pose.position[2]:.3f} m")

            position = pose.position[:2]
            goal_distance = float(np.linalg.norm(goal - position))
            if goal_distance <= args.goal_tolerance:
                print(f"NAVIGATION_GOAL_REACHED distance={goal_distance:.3f}", flush=True)
                break

            points = depth_to_world_points(depth, max_depth=args.max_depth)
            if points.size:
                distances = np.linalg.norm(points[:, :2] - position, axis=1)
                points = points[distances > args.self_filter_radius]
            if points.size:
                clearance = float(np.min(np.linalg.norm(points[:, :2] - position, axis=1)))
                if clearance < args.critical_clearance:
                    raise RuntimeError(f"Obstaculo a distancia critica: {clearance:.3f} m")
            else:
                clearance = math.inf
            grid = build_occupancy_grid(
                points,
                position,
                goal,
                resolution=args.resolution,
                inflation_radius=args.inflation_radius,
            )
            waypoint_reached = waypoint is not None and np.linalg.norm(waypoint - position) <= 0.2
            waypoint_blocked = False
            if waypoint is not None:
                waypoint_cell = grid.world_to_cell(waypoint)
                waypoint_blocked = (
                    not 0 <= waypoint_cell[0] < grid.occupied.shape[0]
                    or not 0 <= waypoint_cell[1] < grid.occupied.shape[1]
                    or grid.occupied[waypoint_cell]
                )
            if waypoint is None or waypoint_reached or waypoint_blocked:
                path = astar(grid, position, goal)
                if not path:
                    raise RuntimeError("A* no encontro una ruta segura")
                waypoint = select_waypoint(path, position, args.lookahead)
                path_cells = len(path)
            desired_yaw = math.atan2(waypoint[1] - position[1], waypoint[0] - position[0])
            elapsed = max(now - last_update, PUBLISH_PERIOD)
            max_yaw_step = math.radians(30.0) * elapsed
            yaw_step = float(np.clip(angle_difference(desired_yaw, commanded_yaw), -max_yaw_step, max_yaw_step))
            commanded_yaw += yaw_step
            measured_yaw = yaw_from_pose(pose)
            aligned = abs(angle_difference(desired_yaw, measured_yaw)) <= math.radians(15.0)
            movement = (
                (
                    args.speed * math.cos(desired_yaw),
                    args.speed * math.sin(desired_yaw),
                    0.0,
                )
                if aligned
                else (0.0, 0.0, 0.0)
            )
            facing = (math.cos(commanded_yaw), math.sin(commanded_yaw), 0.0)
            publisher.send(planner_message(WALK, movement, facing, -1.0))
            safe_idle = planner_message(IDLE, (0.0, 0.0, 0.0), facing, -1.0)
            if now - last_report >= 1.0:
                print(
                    f"NAVIGATION_PROGRESS x={position[0]:.2f} y={position[1]:.2f} "
                    f"goal={goal_distance:.2f} clearance={clearance:.2f} path_cells={path_cells}",
                    flush=True,
                )
                last_report = now
            last_update = now
            time.sleep(PUBLISH_PERIOD)
    except Exception as exc:
        print(f"NAVIGATION_STOP {exc}", flush=True)
        raise
    finally:
        if activated and safe_idle is not None:
            publish_for(publisher, safe_idle, 1.0)
        depth_subscriber.close()
        telemetry_subscriber.close()
        pose_subscriber.close()
        publisher.close()
        context.term()


if __name__ == "__main__":
    main()