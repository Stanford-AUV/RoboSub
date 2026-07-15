#!/usr/bin/env bash
# Run on the SUB (Orin). One command to drive the sub with a laptop controller:
# hardware stack + nats-server + manual/joystick node + a power/thruster plot
# (joystick.png). Pilot laptop runs robosub_local/run.sh --host <sub tether IP>.
#
#   ./teleop_remote.sh                       # full bringup
#   ./teleop_remote.sh -p wrench_scale:=1.5  # softer push (args -> joystick node)
#   NO_HARDWARE=1 ./teleop_remote.sh         # skip hardware (running it elsewhere)
#
# Ctrl+C kills the hardware group and drives neutral PWM (1500) to every
# thruster so nothing latches spinning (same safety as launch_sub.sh).
# No `set -u`: conda's ROS activate.d references unbound vars.
set -o pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# thruster-safety config (mirrors launch_sub.sh)
ARDUINO_PORT="${ARDUINO_PORT:-/dev/ttyACM_teensy}"
ARDUINO_BAUD=9600
THRUSTER_COUNT=8
NEUTRAL_PWM=1500
HEARTBEAT_MATCH="orin_heartbeat.py"
HW_PID=""; JOY_PID=""; NATS_PID=""; PLOT_PID=""; KILLED=0

# conda robosub env + workspace
source "${CONDA_SH:-$HOME/miniforge3/etc/profile.d/conda.sh}" || { echo "ERROR: conda.sh" >&2; exit 1; }
[ "${CONDA_DEFAULT_ENV:-}" != "robosub" ] && { conda activate robosub || { echo "ERROR: activate robosub" >&2; exit 1; }; }
source "$ROOT/install/setup.bash"

# Send neutral PWM to the thrusters (compact copy of launch_sub.sh kill_thrusters).
kill_thrusters() {
    local msg; msg="$(python3 -c "print(' '.join(['${NEUTRAL_PWM}']*${THRUSTER_COUNT}))")"
    for _ in $(seq 1 20); do
        command -v fuser >/dev/null 2>&1 && fuser -s "$ARDUINO_PORT" 2>/dev/null && sleep 0.1 || break
    done
    [ -e "$ARDUINO_PORT" ] || { echo "WARNING: $ARDUINO_PORT absent; no neutral PWM." >&2; return 1; }
    echo ">>> Neutral PWM ($NEUTRAL_PWM x $THRUSTER_COUNT) -> $ARDUINO_PORT"
    ARDUINO_PORT="$ARDUINO_PORT" ARDUINO_BAUD="$ARDUINO_BAUD" MSG="$msg" python3 <<'PY'
import os, time, sys
try: import serial
except ImportError: sys.stderr.write("pyserial missing\n"); sys.exit(1)
try:
    with serial.Serial(os.environ["ARDUINO_PORT"], int(os.environ["ARDUINO_BAUD"]), timeout=1, exclusive=True) as ser:
        time.sleep(0.2)
        for _ in range(10): ser.write((os.environ["MSG"]+"\n").encode()); ser.flush(); time.sleep(0.05)
    print("Thrusters neutralized.")
except Exception as e: sys.stderr.write(f"neutral PWM failed: {e}\n"); sys.exit(1)
PY
}

kill_group() {  # SIGINT a process group, escalate if it lingers
    kill -INT "-$1" 2>/dev/null
    for _ in $(seq 1 40); do kill -0 "-$1" 2>/dev/null || return 0; sleep 0.2; done
    kill -TERM "-$1" 2>/dev/null; sleep 1; kill -KILL "-$1" 2>/dev/null
}

shutdown() {
    [ "$KILLED" -eq 1 ] && return; KILLED=1
    echo; echo ">>> Ctrl+C - killing thrusters first..."
    pkill -STOP -f "$HEARTBEAT_MATCH" 2>/dev/null
    # -PID targets the whole process group so ros2-run's python child dies too.
    [ "$JOY_PID" ] && kill -INT "-$JOY_PID" 2>/dev/null
    [ "$PLOT_PID" ] && kill -INT "-$PLOT_PID" 2>/dev/null
    [ "$HW_PID" ] && kill_group "$HW_PID"
    kill_thrusters
    pkill -CONT -f "$HEARTBEAT_MATCH" 2>/dev/null
    [ "$NATS_PID" ] && kill "$NATS_PID" 2>/dev/null
    [ "$JOY_PID" ] && kill -KILL "-$JOY_PID" 2>/dev/null
    [ "$PLOT_PID" ] && kill -KILL "-$PLOT_PID" 2>/dev/null
    echo ">>> Done."; exit 0
}
trap shutdown INT TERM

# Node output -> log files (an SSH-pty write can block a node's executor if the
# tether stalls -> latched PWMs; launch_sub.sh logs to files for this reason).
LOG_DIR="${ROOT}/logs/teleop_$(date +%Y%m%d_%H%M%S)"; mkdir -p "$LOG_DIR"
echo ">>> Node output -> $LOG_DIR/  (tail -f to watch)"

# nats-server (reuse if already listening)
if ss -ltn 2>/dev/null | grep -q ':4222'; then
    echo ">>> nats-server already on :4222 - reusing."
else
    echo ">>> Starting nats-server ..."; "$ROOT/tools/nats_server.sh" >"$LOG_DIR/nats.log" 2>&1 & NATS_PID=$!; sleep 1
fi

set -m   # each background launch gets its own process group

# hardware stack (localization plot off; teleop uses the simplified joystick.png plot)
if [ -z "${NO_HARDWARE:-}" ]; then
    echo ">>> ros2 launch main hardware.py plot:=false  (log: $LOG_DIR/hardware.log)"
    ros2 launch main hardware.py plot:=false </dev/null >"$LOG_DIR/hardware.log" 2>&1 & HW_PID=$!
    echo ">>> Waiting 8s for thrust_generator + arduino..."; for _ in $(seq 1 8); do [ "$KILLED" -eq 1 ] && break; sleep 1; done
else
    echo ">>> NO_HARDWARE=1 - assuming hardware.py runs elsewhere."
fi

# power/thruster plot -> joystick.png
[ "$KILLED" -eq 1 ] && shutdown
echo ">>> Plot -> $ROOT/joystick.png  (log: $LOG_DIR/plot.log)"
SENSORS_PLOT_OUT=joystick.png ros2 run hardware sensors_plot_simplified </dev/null >"$LOG_DIR/plot.log" 2>&1 & PLOT_PID=$!

# manual/joystick node
echo ">>> Starting manual/joystick node  (log: $LOG_DIR/joystick.log)"
echo ">>> Pilot laptop: robosub_local/run.sh --host <this sub's tether IP>"
ros2 run manual joystick --ros-args "$@" </dev/null >"$LOG_DIR/joystick.log" 2>&1 & JOY_PID=$!

echo ">>> Teleop up. Ctrl+C to stop and neutralize thrusters."
wait -n 2>/dev/null || true
shutdown
