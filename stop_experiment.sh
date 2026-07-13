#!/usr/bin/env bash
# Tear down the imu-thrust experiment and neutralize the thrusters.
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
tmux kill-session -t cpu_imu_exp 2>/dev/null && echo ">>> tmux session killed."
# kill_sub.sh closes the bag cleanly, sends neutral PWM (1500 x8), and sweeps
# any surviving node processes.
exec "$REPO/kill_sub.sh"
