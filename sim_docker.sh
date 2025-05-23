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

export PYTHONPATH="$(pwd)/src/simulation:$PYTHONPATH"

# nats-server &
# PIDS+=("$!")

/home/ros/env/bin/python src/simulation/simulation/bridge_docker.py &
PIDS+=("$!")

ros2 run ros_gz_bridge parameter_bridge \
     --ros-args -p config_file:=src/simulation/simulation/gazebo_bridge.yaml &
PIDS+=("$!")

ros2 launch src/main/launch/simulation.py &
PIDS+=("$!")

echo "[main] Running... Press Ctrl+C to stop."
wait -n
