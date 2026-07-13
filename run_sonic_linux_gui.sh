#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SONIC_ROOT="${SONIC_ROOT:-$(cd "$SCRIPT_DIR/.." && pwd)}"
VIEWER_ROOT="${SONIC_VIEWER_ROOT:-$SCRIPT_DIR}"
PYTHON="${SONIC_PYTHON:-$SONIC_ROOT/.venv_sim/bin/python}"
DISPLAY="${DISPLAY:-:0}"
XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
MODEL="${SONIC_MODEL:-$VIEWER_ROOT/model/g1_29dof_with_hand_rev_1_0.xml}"

export DISPLAY XAUTHORITY

cd "$VIEWER_ROOT"
exec "$PYTHON" sonic_viewer.py \
  --url tcp://127.0.0.1:5557 \
  --physical-url tcp://127.0.0.1:5558 \
  --model "$MODEL"