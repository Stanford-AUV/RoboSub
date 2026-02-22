#!/usr/bin/env bash
set -Eeuo pipefail

# Store all child PIDs here
PIDS=()

cleanup() {
    echo -e "\n[cleanup] Cleaning up processes: ${PIDS[*]}"

    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            echo "[cleanup] Killing tree rooted at PID $pid"
            # This kills the whole tree below each PID
            pkill -TERM -P "$pid" || true
            kill -TERM "$pid"     || true
        fi
    done

    sleep 1

    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            echo "[cleanup] Force-killing PID $pid"
            pkill -KILL -P "$pid" || true
            kill -KILL "$pid"     || true
        fi
    done

    echo "[cleanup] Done."
}
trap cleanup EXIT INT TERM HUP

# Gracefully handle unset PYTHONPATH or GZ_SIM_RESOURCE_PATH
export PYTHONPATH="${PYTHONPATH:-}"
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-}"

# Set environment variables
export PYTHONPATH="$(pwd)/src/simulation:$PYTHONPATH"
export GZ_SIM_RESOURCE_PATH="$(pwd)/src/simulation/simulation/models:$GZ_SIM_RESOURCE_PATH"
# export GZ_SIM_SYSTEM_PLUGIN_PATH="$(pwd)/build/custom_gz_plugins:$GZ_SIM_SYSTEM_PLUGIN_PATH"
# export GZ_GUI_PLUGIN_PATH="$(pwd)/build/custom_gz_plugins:$GZ_GUI_PLUGIN_PATH"

gz sim -s src/simulation/simulation/models/world.urdf &
PIDS+=("$!")

gz sim -g &
PIDS+=("$!")

./.local_venv/bin/python3 src/simulation/simulation/bridge_local.py &
PIDS+=("$!")

echo "[main] Running... Press Ctrl+C to stop."
wait
