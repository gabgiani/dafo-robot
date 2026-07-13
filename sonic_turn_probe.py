from __future__ import annotations

import argparse
import math
import socket
import time
from urllib.parse import urlparse

import zmq

from sonic_remote_control import IDLE, command_message, planner_message
from sonic_telemetry import SonicPhysicalPose, SonicPhysicalPoseSubscriber, SonicTelemetrySubscriber


WALK = 2
PUBLISH_PERIOD = 0.05


def yaw_from_pose(pose: SonicPhysicalPose) -> float:
    qw, qx, qy, qz = pose.quaternion
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def wait_for_pose(subscriber: SonicPhysicalPoseSubscriber, timeout: float) -> SonicPhysicalPose:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        error = subscriber.error()
        if error is not None:
            raise error
        pose = subscriber.latest()
        if pose is not None and time.monotonic() - pose.received_at < 0.5:
            return pose
        time.sleep(0.02)
    raise TimeoutError("No se recibio odometria fisica reciente")


def wait_for_endpoint(url: str, timeout: float) -> None:
    parsed = urlparse(url)
    if parsed.scheme != "tcp" or parsed.hostname is None or parsed.port is None:
        raise ValueError(f"Endpoint de telemetria no valido: {url}")
    deadline = time.monotonic() + timeout
    last_error: OSError | None = None
    while time.monotonic() < deadline:
        try:
            with socket.create_connection((parsed.hostname, parsed.port), timeout=0.2):
                return
        except OSError as exc:
            last_error = exc
            time.sleep(0.02)
    raise TimeoutError(f"SONIC no escucha en {url}; no se activara el control") from last_error


def wait_for_telemetry(subscriber: SonicTelemetrySubscriber, timeout: float) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        error = subscriber.error()
        if error is not None:
            raise error
        pose = subscriber.latest()
        if pose is not None and time.monotonic() - pose.received_at < 0.5:
            return
        time.sleep(0.02)
    raise TimeoutError("SONIC no publico telemetria despues de activar el control")


def publish_for(publisher: zmq.Socket, message: bytes, duration: float) -> None:
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        publisher.send(message)
        time.sleep(PUBLISH_PERIOD)


def main() -> None:
    parser = argparse.ArgumentParser(description="Mide el giro fisico en sitio del planner SONIC")
    parser.add_argument("--bind", default="tcp://127.0.0.1:5556")
    parser.add_argument("--pose-url", default="tcp://127.0.0.1:5558")
    parser.add_argument("--telemetry-url", default="tcp://127.0.0.1:5557")
    parser.add_argument("--angle", type=float, default=90.0, help="Giro solicitado en grados")
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--pose-timeout", type=float, default=5.0)
    args = parser.parse_args()
    if not -180.0 <= args.angle <= 180.0 or args.angle == 0.0:
        parser.error("--angle debe estar entre -180 y 180 grados y no puede ser cero")
    if args.duration <= 0.0:
        parser.error("--duration debe ser mayor que cero")

    context = zmq.Context()
    publisher = context.socket(zmq.PUB)
    publisher.setsockopt(zmq.LINGER, 0)
    publisher.bind(args.bind)
    pose_subscriber = SonicPhysicalPoseSubscriber(args.pose_url)
    telemetry_subscriber = SonicTelemetrySubscriber(args.telemetry_url)
    pose_subscriber.start()
    telemetry_subscriber.start()
    activated = False
    safe_idle: bytes | None = None

    try:
        time.sleep(0.6)
        wait_for_endpoint(args.telemetry_url, args.pose_timeout)
        initial_pose = wait_for_pose(pose_subscriber, args.pose_timeout)
        initial_yaw = yaw_from_pose(initial_pose)
        initial_facing = (math.cos(initial_yaw), math.sin(initial_yaw), 0.0)
        idle = planner_message(IDLE, (0.0, 0.0, 0.0), initial_facing, -1.0)
        safe_idle = idle

        activation_deadline = time.monotonic() + 2.0
        activated = True
        while time.monotonic() < activation_deadline:
            publisher.send(command_message(start=True, stop=False))
            publisher.send(idle)
            time.sleep(PUBLISH_PERIOD)
        wait_for_telemetry(telemetry_subscriber, args.pose_timeout)

        target_yaw = initial_yaw + math.radians(args.angle)
        started_at = time.monotonic()
        while True:
            elapsed = time.monotonic() - started_at
            progress = min(1.0, elapsed / args.duration)
            commanded_yaw = initial_yaw + (target_yaw - initial_yaw) * progress
            facing = (math.cos(commanded_yaw), math.sin(commanded_yaw), 0.0)
            publisher.send(planner_message(WALK, (0.0, 0.0, 0.0), facing, -1.0))
            if progress >= 1.0:
                break
            time.sleep(PUBLISH_PERIOD)

        target_facing = (math.cos(target_yaw), math.sin(target_yaw), 0.0)
        safe_idle = planner_message(IDLE, (0.0, 0.0, 0.0), target_facing, -1.0)
        publish_for(
            publisher,
            safe_idle,
            0.5,
        )
        final_pose = wait_for_pose(pose_subscriber, args.pose_timeout)
        final_yaw = yaw_from_pose(final_pose)
        measured_delta = math.atan2(
            math.sin(final_yaw - initial_yaw),
            math.cos(final_yaw - initial_yaw),
        )
        print(f"yaw inicial: {math.degrees(initial_yaw):+.2f} deg")
        print(f"yaw objetivo: {math.degrees(target_yaw):+.2f} deg")
        print(f"yaw final: {math.degrees(final_yaw):+.2f} deg")
        print(f"giro fisico medido: {math.degrees(measured_delta):+.2f} deg")
    finally:
        if activated and safe_idle is not None:
            publish_for(publisher, safe_idle, 1.0)
        telemetry_subscriber.close()
        pose_subscriber.close()
        publisher.close()
        context.term()


if __name__ == "__main__":
    main()