from __future__ import annotations

from dataclasses import dataclass
import json
import socket
from typing import Any


@dataclass
class ExternalControlCommand:
    payload: dict[str, Any]
    sender: tuple[str, int]


class UdpExternalControl:
    def __init__(self, host: str, port: int):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind((host, port))
        self._socket.setblocking(False)

    @property
    def address(self) -> tuple[str, int]:
        host, port = self._socket.getsockname()
        return str(host), int(port)

    def poll(self) -> list[ExternalControlCommand]:
        commands: list[ExternalControlCommand] = []
        while True:
            try:
                packet, sender = self._socket.recvfrom(65535)
            except BlockingIOError:
                break
            except OSError:
                break

            if not packet:
                continue

            try:
                payload = json.loads(packet.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                print(f"[control] paquete invalido desde {sender}: {exc}")
                continue

            if not isinstance(payload, dict):
                print(f"[control] ignorando payload no-objeto desde {sender}: {payload!r}")
                continue

            commands.append(ExternalControlCommand(payload=payload, sender=sender))

        return commands

    def close(self) -> None:
        self._socket.close()

    def __enter__(self) -> UdpExternalControl:
        return self

    def __exit__(self, exc_type: object, exc: object, tb: object) -> None:
        self.close()


def send_udp_command(host: str, port: int, payload: dict[str, Any]) -> None:
    packet = json.dumps(payload, separators=(",", ":")).encode("utf-8")
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.sendto(packet, (host, port))