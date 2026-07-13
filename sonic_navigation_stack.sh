#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SONIC_ROOT="${SONIC_ROOT:-$(cd -- "$SCRIPT_DIR/.." && pwd)}"
SONIC_PYTHON="${SONIC_PYTHON:-$SONIC_ROOT/.venv_sim/bin/python}"
DEPLOY_DIR="$SONIC_ROOT/gear_sonic_deploy"
SCENE_SOURCE="$SCRIPT_DIR/sonic_navigation_scene.xml"
SCENE_TARGET="$SONIC_ROOT/gear_sonic/data/robot_model/model_data/g1/sonic_navigation_scene.xml"
MOTION_DIR="${SONIC_MOTION_DIR:-/tmp/sonic_smoke_motion}"
LOG_DIR="${SONIC_LOG_DIR:-/tmp}"
SIM_LOG="$LOG_DIR/sonic_navigation_sim.log"
RELAY_LOG="$LOG_DIR/sonic_navigation_relay.log"
POLICY_LOG="$LOG_DIR/sonic_navigation_policy.log"
CONTROL_LOG="$LOG_DIR/sonic_navigation_control.log"
SIM_PID=""
RELAY_PID=""
POLICY_PID=""

wait_for_log() {
    local file="$1"
    local marker="$2"
    local timeout="$3"
    local deadline=$((SECONDS + timeout))
    while (( SECONDS < deadline )); do
        if grep -qF "$marker" "$file" 2>/dev/null; then
            return 0
        fi
        sleep 0.1
    done
    return 1
}

wait_for_port() {
    local port="$1"
    local timeout="$2"
    local deadline=$((SECONDS + timeout))
    while (( SECONDS < deadline )); do
        if "$SONIC_PYTHON" - "$port" <<'PY'
import socket
import sys

with socket.socket() as connection:
    connection.settimeout(0.1)
    raise SystemExit(connection.connect_ex(("127.0.0.1", int(sys.argv[1]))))
PY
        then
            return 0
        fi
        sleep 0.1
    done
    return 1
}

stop_stack() {
    local pid
    for pid in "$POLICY_PID" "$RELAY_PID" "$SIM_PID"; do
        if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
            kill -TERM "$pid" 2>/dev/null || true
        fi
    done
    for pid in "$POLICY_PID" "$RELAY_PID" "$SIM_PID"; do
        if [[ -n "$pid" ]]; then
            for _attempt in {1..50}; do
                kill -0 "$pid" 2>/dev/null || break
                sleep 0.1
            done
            kill -KILL "$pid" 2>/dev/null || true
            wait "$pid" 2>/dev/null || true
        fi
    done
}
trap stop_stack EXIT INT TERM

if [[ "$(uname -s)" != "Linux" ]]; then
    echo "This launcher is supported only on the remote Linux host." >&2
    exit 1
fi
for required in "$SONIC_PYTHON" "$SCENE_SOURCE" "$DEPLOY_DIR/target/release/g1_deploy_onnx_ref"; do
    if [[ ! -e "$required" ]]; then
        echo "Required path not found: $required" >&2
        exit 1
    fi
done
if [[ ! -d "$MOTION_DIR" ]] || ! find "$MOTION_DIR" -mindepth 1 -maxdepth 1 -print -quit | grep -q .; then
    echo "Motion directory is missing or empty: $MOTION_DIR" >&2
    exit 1
fi
if [[ -z "${DISPLAY:-}" || -z "${XAUTHORITY:-}" ]]; then
    echo "DISPLAY and XAUTHORITY must identify the active Linux desktop session." >&2
    exit 1
fi

install -m 0644 "$SCENE_SOURCE" "$SCENE_TARGET"
mkdir -p "$LOG_DIR"
: > "$SIM_LOG"
: > "$RELAY_LOG"
: > "$POLICY_LOG"
: > "$CONTROL_LOG"

cd "$SONIC_ROOT"
PYTHONUNBUFFERED=1 "$SONIC_PYTHON" "$SCRIPT_DIR/sonic_navigation_sim.py" \
    --scene "$SCENE_TARGET" >"$SIM_LOG" 2>&1 &
SIM_PID=$!
if ! wait_for_log "$SIM_LOG" "SONIC_NAVIGATION_SIM_READY" 40; then
    echo "Simulator did not become ready. Full log: $SIM_LOG" >&2
    exit 1
fi

PYTHONUNBUFFERED=1 "$SONIC_PYTHON" "$SCRIPT_DIR/sonic_physical_pose_relay.py" \
    >"$RELAY_LOG" 2>&1 &
RELAY_PID=$!

cd "$DEPLOY_DIR"
./target/release/g1_deploy_onnx_ref lo \
    policy/low_latency/model_decoder.onnx "$MOTION_DIR" \
    --planner-file planner/target_vel/V2/planner_sonic.onnx \
    --planner-precision 16 \
    --obs-config policy/low_latency/observation_config.yaml \
    --encoder-file policy/low_latency/model_encoder.onnx \
    --input-type zmq_manager \
    --zmq-host 127.0.0.1 \
    --zmq-port 5556 \
    --output-type all \
    --disable-crc-check >"$POLICY_LOG" 2>&1 &
POLICY_PID=$!
if ! wait_for_port 5557 60; then
    echo "Policy did not open telemetry port 5557. Full log: $POLICY_LOG" >&2
    exit 1
fi

cd "$SONIC_ROOT"
set +e
"$SONIC_PYTHON" "$SCRIPT_DIR/sonic_autonomous_navigation.py" "$@" \
    2>&1 | tee "$CONTROL_LOG"
control_status=${PIPESTATUS[0]}
set -e
exit "$control_status"
