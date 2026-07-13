#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SONIC_ROOT="${SONIC_ROOT:-$(cd "$SCRIPT_DIR/.." && pwd)}"
GUI_ROOT="${SONIC_GUI_ROOT:-$SCRIPT_DIR}"
PYTHON="${SONIC_PYTHON:-$SONIC_ROOT/.venv_sim/bin/python}"
DISPLAY="${DISPLAY:-:0}"
XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"

export DISPLAY XAUTHORITY

notify() {
    notify-send "SONIC" "$1"
}

start_sim() {
    if pgrep -f "[r]un_sim_loop.py --interface sim" >/dev/null; then
        notify "La simulacion ya esta activa."
        return
    fi

    pkill -f "[s]onic_teleop.py" 2>/dev/null || true
    pkill -f "[g]1_deploy_onnx_ref" 2>/dev/null || true

    rm -f /tmp/sonic_desktop_sim.log
    cd "$SONIC_ROOT"
    setsid -f env PYTHONUNBUFFERED=1 "$PYTHON" \
        gear_sonic/scripts/run_sim_loop.py \
        --interface sim \
        --no-enable-offscreen \
        >/tmp/sonic_desktop_sim.log 2>&1 </dev/null

    for _attempt in {1..300}; do
        if grep -q "SONIC_SIM_READY" /tmp/sonic_desktop_sim.log 2>/dev/null; then
            notify "Simulacion lista. Ya puedes abrir SONIC Control."
            return
        fi
        if ! pgrep -f "[r]un_sim_loop.py --interface sim" >/dev/null; then
            notify "El simulador no pudo iniciar. Revisa /tmp/sonic_desktop_sim.log."
            exit 1
        fi
        sleep 0.1
    done

    notify "El simulador no confirmo que esta listo. Revisa /tmp/sonic_desktop_sim.log."
    exit 1
}

start_control() {
    if ! pgrep -f "[r]un_sim_loop.py --interface sim" >/dev/null; then
        notify "Primero inicia SONIC Simulator."
        exit 1
    fi

    if ! grep -q "SONIC_SIM_READY" /tmp/sonic_desktop_sim.log 2>/dev/null; then
        notify "El simulador aun no esta listo. Espera la confirmacion de SONIC Simulator."
        exit 1
    fi

    if grep -q "Robot has fallen" /tmp/sonic_desktop_sim.log 2>/dev/null; then
        notify "El simulador esta detenido por una caida. Usa SONIC 3 Stop y luego SONIC 1 Simulator."
        exit 1
    fi

    if ! pgrep -f "[g]1_deploy_onnx_ref" >/dev/null; then
        rm -f /tmp/sonic_desktop_policy.log
        cd "$SONIC_ROOT/gear_sonic_deploy"
        setsid -f ./target/release/g1_deploy_onnx_ref \
            lo \
            policy/low_latency/model_decoder.onnx \
            /tmp/sonic_smoke_motion \
            --planner-file planner/target_vel/V2/planner_sonic.onnx \
            --planner-precision 16 \
            --obs-config policy/low_latency/observation_config.yaml \
            --encoder-file policy/low_latency/model_encoder.onnx \
            --input-type zmq_manager \
            --zmq-host 127.0.0.1 \
            --zmq-port 5556 \
            --output-type all \
            --disable-crc-check \
            >/tmp/sonic_desktop_policy.log 2>&1 </dev/null

        for _attempt in {1..30}; do
            if pgrep -f "[g]1_deploy_onnx_ref" >/dev/null; then
                break
            fi
            sleep 0.1
        done
        if ! pgrep -f "[g]1_deploy_onnx_ref" >/dev/null; then
            notify "El policy no pudo iniciar. Revisa /tmp/sonic_desktop_policy.log."
            exit 1
        fi
    fi

    if pgrep -f "[s]onic_teleop.py --bind tcp://127.0.0.1:5556" >/dev/null; then
        notify "El control remoto ya esta abierto."
        return
    fi

    cd "$GUI_ROOT"
    setsid -f "$PYTHON" sonic_teleop.py \
        --bind tcp://127.0.0.1:5556 \
        --initial-arm-mode normal \
        >/tmp/sonic_desktop_teleop.log 2>&1 </dev/null
    notify "Control abierto. Usa Activar en la ventana cuando estes listo."
}

stop_all() {
    pkill -f "[s]onic_teleop.py" 2>/dev/null || true
    pkill -f "[g]1_deploy_onnx_ref" 2>/dev/null || true
    pkill -f "[r]un_sim_loop.py" 2>/dev/null || true
    notify "Simulacion y control detenidos."
}

case "${1:-}" in
    sim)
        start_sim
        ;;
    control)
        start_control
        ;;
    stop)
        stop_all
        ;;
    *)
        printf 'Uso: %s {sim|control|stop}\n' "$0" >&2
        exit 2
        ;;
esac