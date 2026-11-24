#!/usr/bin/env bash
#
# Poolside pre-flight check: tests every sensor, motor and device and prints a
# GO / NO-GO report before the sub goes in the water. See tools/preflight.py.
#
#   ./preflight.sh                 # full hardware check (stack must be STOPPED)
#   ./preflight.sh --live          # stack running (e.g. ./launch_sub.sh --no-thrust)
#   ./preflight.sh --yes           # no prompts; thrusters judged by current draw
#   ./preflight.sh --no-thrusters  # never spin a motor
#   ./preflight.sh --only teensy,dvl   /   --skip cameras
#   ./preflight.sh --in-water      # floating: expect DVL bottom lock
#   ./preflight.sh --help
#
# Exit code: 0 GO, 1 NO-GO, 2 GO with warnings.

# shellcheck disable=SC1091
source "${CONDA_SH:-$HOME/miniforge3/etc/profile.d/conda.sh}" || {
    echo "ERROR: could not source conda.sh (set CONDA_SH to its path)." >&2
    exit 1
}
if [ "${CONDA_DEFAULT_ENV:-}" != "robosub" ]; then
    conda activate robosub || { echo "ERROR: 'conda activate robosub' failed." >&2; exit 1; }
fi

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Same DDS settings as launch_sub.sh so --live sees the running stack.
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-${REPO_DIR}/config/fastdds_async.xml}"
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

if [ -f "${REPO_DIR}/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "${REPO_DIR}/install/setup.bash"
else
    echo "WARNING: workspace not built - ROS-based checks (IMU, --live) will be skipped." >&2
fi

exec python "${REPO_DIR}/tools/preflight.py" "$@"
