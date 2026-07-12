from __future__ import annotations

import argparse
from dataclasses import dataclass
import select
import sys
import termios
import tty

from external_control import send_udp_command


_HELP_TEXT = """Teleop Unitree
  W/S o flecha arriba/abajo: avance +/-
  A/D o flecha izquierda/derecha: giro +/-
  Espacio: centrar avance y giro  G: marcha en el sitio (levantar piernas sin avanzar)    I: imprimir snapshot de sensores
  R: resetear robot
  P: pausar simulacion
  O: reanudar simulacion
  J/K: amplitud -/+
  N/M: frecuencia -/+
  Q: salir
"""


@dataclass
class TeleopState:
    advance: float = 0.0
    turn: float = 0.0
    march: float = 0.0
    amplitude: float = 1.0
    frequency: float = 1.0
    paused: bool = False


class RawTerminal:
    def __enter__(self) -> RawTerminal:
        self._fd = sys.stdin.fileno()
        self._old_attrs = termios.tcgetattr(self._fd)
        tty.setraw(self._fd)
        return self

    def __exit__(self, exc_type: object, exc: object, tb: object) -> None:
        termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_attrs)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Teleoperacion por teclado para Unitree sobre UDP.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=47001)
    parser.add_argument("--axis-step", type=float, default=0.2, help="Incremento por tecla para avance y giro.")
    parser.add_argument("--amplitude-step", type=float, default=0.1, help="Incremento por tecla para amplitud.")
    parser.add_argument("--frequency-step", type=float, default=0.1, help="Incremento por tecla para frecuencia.")
    return parser.parse_args()


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def read_key() -> str:
    first = sys.stdin.read(1)
    if first != "\x1b":
        return first
    ready, _, _ = select.select([sys.stdin], [], [], 0.01)
    if not ready:
        return first
    second = sys.stdin.read(1)
    if second != "[":
        return first + second
    third = sys.stdin.read(1)
    return first + second + third


def print_status(state: TeleopState) -> None:
    print(
        f"\radvance={state.advance:+.2f} turn={state.turn:+.2f} "
        f"amp={state.amplitude:.2f} freq={state.frequency:.2f} paused={state.paused}   ",
        end="",
        flush=True,
    )


def send_state(host: str, port: int, state: TeleopState) -> None:
    send_udp_command(
        host,
        port,
        {
            "advance": state.advance,
            "turn": state.turn,
            "amplitude": state.amplitude,
            "frequency": state.frequency,
            "paused": state.paused,
        },
    )


def handle_key(
    key: str,
    state: TeleopState,
    axis_step: float,
    amplitude_step: float,
    frequency_step: float,
) -> dict[str, object] | None:
    lower = key.lower()
    if key in ("w", "W", "\x1b[A"):
        state.advance = clamp(state.advance + axis_step, -1.0, 1.0)
    elif key in ("s", "S", "\x1b[B"):
        state.advance = clamp(state.advance - axis_step, -1.0, 1.0)
    elif key in ("a", "A", "\x1b[D"):
        state.turn = clamp(state.turn + axis_step, -1.0, 1.0)
    elif key in ("d", "D", "\x1b[C"):
        state.turn = clamp(state.turn - axis_step, -1.0, 1.0)
    elif key == " ":
        state.advance = 0.0
        state.turn = 0.0
        return {"center": True}
    elif lower == "g":
        state.march = 0.0 if state.march > 0.5 else 1.0
        return {"march": state.march}
    elif lower == "i":
        return {"report_sensors": True}
    elif lower == "r":
        state.advance = 0.0
        state.turn = 0.0
        return {"reset": True}
    elif lower == "p":
        state.paused = True
    elif lower == "o":
        state.paused = False
    elif lower == "j":
        state.amplitude = clamp(state.amplitude - amplitude_step, 0.2, 1.2)
    elif lower == "k":
        state.amplitude = clamp(state.amplitude + amplitude_step, 0.2, 1.2)
    elif lower == "n":
        state.frequency = clamp(state.frequency - frequency_step, 0.8, 2.2)
    elif lower == "m":
        state.frequency = clamp(state.frequency + frequency_step, 0.8, 2.2)
    elif lower == "q":
        raise KeyboardInterrupt
    else:
        return None

    return {
        "advance": state.advance,
        "turn": state.turn,
        "amplitude": state.amplitude,
        "frequency": state.frequency,
        "paused": state.paused,
    }


def main() -> int:
    args = parse_args()
    state = TeleopState()
    print(_HELP_TEXT)
    print_status(state)

    try:
        with RawTerminal():
            while True:
                key = read_key()
                payload = handle_key(
                    key,
                    state,
                    args.axis_step,
                    args.amplitude_step,
                    args.frequency_step,
                )
                if payload is None:
                    continue
                send_udp_command(args.host, args.port, payload)
                print_status(state)
    except KeyboardInterrupt:
        print("\nteleop cerrado")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())