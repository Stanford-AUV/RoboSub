#!/usr/bin/env bash
# Record raw /imu/data overnight for Allan variance analysis.
#
# Usage (run inside tmux, e.g. `tmux new -s cpu_allan`):
#   conda activate robosub
#   ./record.sh [hours]     # default 12
#
# Requirements before starting:
#   - Xsens driver running and /imu/data publishing
#   - Vehicle/IMU perfectly static on a solid surface, nobody touches it
set -euo pipefail

HOURS="${1:-12}"
DIR="$(cd "$(dirname "$0")" && pwd)/data"
NAME="imu_static_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$DIR"

echo "Recording /imu/data for ${HOURS}h -> ${DIR}/${NAME}"
echo "Do NOT touch the vehicle until this exits."

exec timeout "${HOURS}h" ros2 bag record /imu/data -o "${DIR}/${NAME}"
