from __future__ import annotations

import argparse

from external_control import send_udp_command


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Envia comandos UDP al simulador Unitree.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=47001)
    parser.add_argument("--advance", type=float, help="Objetivo de avance en [-1, 1].")
    parser.add_argument("--turn", type=float, help="Objetivo de giro en [-1, 1].")
    parser.add_argument("--amplitude", type=float, help="Amplitud absoluta en [0.2, 1.2].")
    parser.add_argument("--frequency", type=float, help="Frecuencia absoluta en [0.8, 2.2].")
    parser.add_argument("--center", action="store_true", help="Centra avance y giro.")
    parser.add_argument("--reset", action="store_true", help="Reinicia el robot al keyframe inicial.")
    parser.add_argument("--pause", action="store_true", help="Pausa la simulacion.")
    parser.add_argument("--resume", action="store_true", help="Reanuda la simulacion.")
    return parser.parse_args()


def build_payload(args: argparse.Namespace) -> dict[str, object]:
    payload: dict[str, object] = {}
    if args.advance is not None:
        payload["advance"] = args.advance
    if args.turn is not None:
        payload["turn"] = args.turn
    if args.amplitude is not None:
        payload["amplitude"] = args.amplitude
    if args.frequency is not None:
        payload["frequency"] = args.frequency
    if args.center:
        payload["center"] = True
    if args.reset:
        payload["reset"] = True
    if args.pause:
        payload["paused"] = True
    if args.resume:
        payload["paused"] = False
    return payload


def main() -> int:
    args = parse_args()
    payload = build_payload(args)
    if not payload:
        raise SystemExit("No hay ningun comando para enviar.")
    send_udp_command(args.host, args.port, payload)
    print(f"comando enviado a {args.host}:{args.port}: {payload}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())