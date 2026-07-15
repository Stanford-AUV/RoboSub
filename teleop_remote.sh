#!/usr/bin/env bash
# Run on the SUB (Orin). ONE command to drive the sub with a laptop controller:
# brings up the hardware stack (thrust_generator + thrusters + arduino + depth +
# dvl), the NATS broker, and the manual/joystick node. A pilot laptop then runs
# robosub_local/run.sh --host <this sub's tether IP>.
#
#   ./teleop_remote.sh                       # full bringup
#   ./teleop_remote.sh -p wrench_scale:=1.5  # softer push (args -> joystick node)
#   NO_HARDWARE=1 ./teleop_remote.sh         # skip hardware (running it elsewhere)
#   PLOT=1 ./teleop_remote.sh                # also start sensors_plot GUI
#
# On Ctrl+C it kills the hardware group FIRST and drives neutral PWM (1500) onto
# every thruster so nothing latches spinning, then tears down nats + the node.
# (Same thruster-safety as launch_sub.sh; do NOT run this together with
# launch_sub.sh / main.py — they each bring up hardware + thrust_generator.)
#
# No `set -u`: RoboStack's conda ROS activate.d references unbound vars
# (CONDA_BUILD) and would abort activation (same reason launch_sub.sh avoids it).
set -o pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# --- thruster-safety config (mirrors launch_sub.sh) -------------------------
ARDUINO_PORT="${ARDUINO_PORT:-/dev/ttyACM_teensy}"   # udev symlink pinned to the Teensy
ARDUINO_BAUD=9600
THRUSTER_COUNT=8
NEUTRAL_PWM=1500
HEARTBEAT_MATCH="orin_heartbeat.py" # systemd heartbeat shares the arduino port
HW_PID=""
JOY_PID=""
NATS_PID=""
KILLED=0

# --- conda robosub env (provides ROS 2) -------------------------------------
# shellcheck disable=SC1091
source "${CONDA_SH:-$HOME/miniforge3/etc/profile.d/conda.sh}" || {
    echo "ERROR: could not source conda.sh (set CONDA_SH)." >&2; exit 1; }
if [ "${CONDA_DEFAULT_ENV:-}" != "robosub" ]; then
    conda activate robosub || { echo "ERROR: conda activate robosub failed." >&2; exit 1; }
fi
# shellcheck disable=SC1091
source "$ROOT/install/setup.bash"

# --- neutralize thrusters (compact copy of launch_sub.sh's kill_thrusters) --
kill_thrusters() {
    local msg
    msg="$(python3 -c "print(' '.join(['${NEUTRAL_PWM}']*${THRUSTER_COUNT}))")"
    # Wait for the arduino node to release the (exclusively-held) port.
    for _ in $(seq 1 20); do
        if command -v fuser >/dev/null 2>&1 && fuser -s "$ARDUINO_PORT" 2>/dev/null; then
            sleep 0.1
        else
            break
        fi
    done
    if [ ! -e "$ARDUINO_PORT" ]; then
        echo "WARNING: $ARDUINO_PORT not present; cannot send neutral PWM." >&2
        return 1
    fi
    echo ">>> Sending neutral PWM ($NEUTRAL_PWM x $THRUSTER_COUNT) to $ARDUINO_PORT"
    ARDUINO_PORT="$ARDUINO_PORT" ARDUINO_BAUD="$ARDUINO_BAUD" MSG="$msg" python3 <<'PY'
import os, time, sys
try:
    import serial
except ImportError:
    sys.stderr.write("pyserial not available; cannot send neutral PWM\n"); sys.exit(1)
port = os.environ["ARDUINO_PORT"]; baud = int(os.environ["ARDUINO_BAUD"])
msg = os.environ["MSG"] + "\n"
try:
    with serial.Serial(port, baudrate=baud, timeout=1, exclusive=True) as ser:
        time.sleep(0.2)
        for _ in range(10):
            ser.write(msg.encode()); ser.flush(); time.sleep(0.05)
    print("Thrusters neutralized.")
except Exception as e:
    sys.stderr.write(f"Failed to send neutral PWM: {e}\n"); sys.exit(1)
PY
}

kill_group() {  # SIGINT a whole process group, escalate if it lingers
    local pid="$1"
    kill -INT "-$pid" 2>/dev/null
    for _ in $(seq 1 40); do kill -0 "-$pid" 2>/dev/null || return 0; sleep 0.2; done
    kill -TERM "-$pid" 2>/dev/null; sleep 1; kill -KILL "-$pid" 2>/dev/null
}

shutdown() {
    [ "$KILLED" -eq 1 ] && return
    KILLED=1
    echo
    echo ">>> Caught Ctrl+C - killing thrusters first..."
    # Freeze the heartbeat so it can't grab the arduino port mid-neutralize.
    pkill -STOP -f "$HEARTBEAT_MATCH" 2>/dev/null && echo ">>> Froze heartbeat"
    [ "$JOY_PID" ] && kill "$JOY_PID" 2>/dev/null           # stop new /wrench first
    [ "$HW_PID" ] && kill_group "$HW_PID"                   # take down arduino node
    kill_thrusters
    pkill -CONT -f "$HEARTBEAT_MATCH" 2>/dev/null && echo ">>> Resumed heartbeat"
    [ "$NATS_PID" ] && kill "$NATS_PID" 2>/dev/null
    echo ">>> Done."
    exit 0
}
trap shutdown INT TERM

# --- logs (NOT the terminal: an SSH pty write can block a node's executor if
# the tether stalls -> latched PWMs; launch_sub.sh logs to files for this) -----
LOG_DIR="${ROOT}/logs/teleop_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"
echo ">>> Node output -> $LOG_DIR/  (tail -f to watch)"

# --- nats-server (reuse if already listening) -------------------------------
if ss -ltn 2>/dev/null | grep -q ':4222'; then
    echo ">>> nats-server already listening on :4222 - reusing it."
else
    echo ">>> Starting nats-server ..."
    "$ROOT/tools/nats_server.sh" >"$LOG_DIR/nats.log" 2>&1 &
    NATS_PID=$!
    sleep 1
fi

# Each background launch/run gets its own process group so shutdown can signal
# the whole group (negative PID) and leave no survivors.
set -m

# --- hardware stack ---------------------------------------------------------
if [ -z "${NO_HARDWARE:-}" ]; then
    PLOT_ARG="plot:=false"; [ "${PLOT:-0}" = "1" ] && PLOT_ARG="plot:=true"
    echo ">>> ros2 launch main hardware.py $PLOT_ARG  (log: $LOG_DIR/hardware.log)"
    ros2 launch main hardware.py "$PLOT_ARG" </dev/null >"$LOG_DIR/hardware.log" 2>&1 &
    HW_PID=$!
    echo ">>> Waiting 8s for thrust_generator + arduino to come up..."
    for _ in $(seq 1 8); do [ "$KILLED" -eq 1 ] && break; sleep 1; done
else
    echo ">>> NO_HARDWARE=1 - assuming hardware.py is already running elsewhere."
fi

# --- manual/joystick node ---------------------------------------------------
[ "$KILLED" -eq 1 ] && shutdown
echo ">>> Starting manual/joystick node  (log: $LOG_DIR/joystick.log)"
echo ">>> Pilot laptop: robosub_local/run.sh --host <this sub's tether IP>"
ros2 run manual joystick --ros-args "$@" </dev/null >"$LOG_DIR/joystick.log" 2>&1 &
JOY_PID=$!

echo ">>> Teleop up. Press Ctrl+C to stop and neutralize thrusters."
wait -n 2>/dev/null || true   # if hardware or the node dies on its own, tear down
shutdown
