#!/usr/bin/env bash
# Run on the SUB (Orin). Starts the NATS broker + the manual/joystick ROS node
# so a pilot laptop can drive the sub over the tether (see robosub_local/).
#
#   ./teleop_remote.sh                       # default push (wrench_scale=2.0)
#   ./teleop_remote.sh -p wrench_scale:=1.5  # softer push (args -> the node)
#
# NOTE: this only handles the teleop bridge. Real thrusters need the hardware
# stack running too:  ros2 launch main hardware.py  (in another terminal).
# On Ctrl+C it stops the joystick node and the nats-server it started.
#
# No `set -u`: RoboStack's conda ROS activate.d references unbound vars
# (CONDA_BUILD) and would abort activation (same reason launch_sub.sh avoids it).
set -o pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# --- conda robosub env (provides ROS 2) ---
# shellcheck disable=SC1091
source "${CONDA_SH:-$HOME/miniforge3/etc/profile.d/conda.sh}" || {
    echo "ERROR: could not source conda.sh (set CONDA_SH)." >&2; exit 1; }
if [ "${CONDA_DEFAULT_ENV:-}" != "robosub" ]; then
    conda activate robosub || { echo "ERROR: conda activate robosub failed." >&2; exit 1; }
fi
# shellcheck disable=SC1091
source "$ROOT/install/setup.bash"

# --- start nats-server if it isn't already listening ---
STARTED_NATS=0
if ss -ltn 2>/dev/null | grep -q ':4222'; then
    echo "nats-server already listening on :4222 — reusing it."
else
    echo "Starting nats-server ..."
    "$ROOT/tools/nats_server.sh" >/dev/null 2>&1 &
    NATS_PID=$!
    STARTED_NATS=1
    sleep 1
fi

cleanup() {
    echo
    if [ "$STARTED_NATS" = "1" ] && kill -0 "${NATS_PID:-0}" 2>/dev/null; then
        echo "Stopping nats-server ($NATS_PID) ..."
        kill "$NATS_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

echo "Starting manual/joystick node. Ctrl+C to stop."
echo "  (pilot laptop: robosub_local/run.sh --host <this sub's tether IP>)"
ros2 run manual joystick --ros-args "$@"
