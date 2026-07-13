#!/usr/bin/env bash
#
# Standalone emergency teardown for the sub stack. Run from ANY terminal
# (including a second one while launch_sub.sh is still up in the first):
#
#     ./kill_sub.sh
#
# Does what launch_sub.sh's Ctrl+C handler does, but self-contained and
# forceful — use it when Ctrl+C didn't fully take the stack down. Order matters:
#   1. Close the rosbag CLEANLY (SIGINT + wait) so metadata.yaml is written and
#      the bag stays readable. A hard kill here leaves an unindexed bag.
#   2. Neutralize thrusters (1500us x8) so the sub can't keep spinning.
#   3. Hard-kill every remaining stack process.
#
set -u

# --- config (override via env) ----------------------------------------------
ARDUINO_PORT="${ARDUINO_PORT:-/dev/ttyACM0}"
ARDUINO_BAUD="${ARDUINO_BAUD:-9600}"
THRUSTER_COUNT="${THRUSTER_COUNT:-8}"
NEUTRAL_PWM="${NEUTRAL_PWM:-1500}"
HEARTBEAT_MATCH="orin_heartbeat.py"

# pyserial (for the neutral-PWM burst) lives in the robosub conda env; pull it in
# if we're not already inside it. Harmless if conda isn't present.
if ! python3 -c "import serial" >/dev/null 2>&1; then
    # shellcheck disable=SC1091
    source ~/miniforge3/etc/profile.d/conda.sh 2>/dev/null && conda activate robosub 2>/dev/null || true
fi

# --- 1. close the rosbag cleanly --------------------------------------------
echo ">>> [1/3] Closing rosbag (SIGINT, waiting for flush)..."
if pgrep -f "bag record" >/dev/null 2>&1; then
    pkill -INT -f "bag record" 2>/dev/null
    for _ in $(seq 1 50); do             # up to ~10s for rosbag2 to write metadata
        pgrep -f "bag record" >/dev/null 2>&1 || break
        sleep 0.2
    done
    if pgrep -f "bag record" >/dev/null 2>&1; then
        echo ">>> rosbag ignored SIGINT after 10s; SIGKILL (bag may be unindexed)"
        pkill -KILL -f "bag record" 2>/dev/null
    else
        echo ">>> rosbag closed cleanly."
    fi
else
    echo ">>> no rosbag running."
fi

# --- 2. neutralize thrusters ------------------------------------------------
echo ">>> [2/3] Neutralizing thrusters..."
# Freeze the Orin->Teensy heartbeat so it can't re-grab the serial port the
# instant the arduino node dies and corrupt our neutral burst.
pkill -STOP -f "$HEARTBEAT_MATCH" 2>/dev/null && echo ">>> froze heartbeat (SIGSTOP)"
# The arduino node holds $ARDUINO_PORT exclusively — kill it so we can open it.
pkill -KILL -f "hardware.nodes.arduino" 2>/dev/null
pkill -KILL -f "install/lib/hardware/arduino" 2>/dev/null

# Wait for the port to actually be released.
for _ in $(seq 1 20); do
    if command -v fuser >/dev/null 2>&1 && fuser -s "$ARDUINO_PORT" 2>/dev/null; then
        sleep 0.1
    else
        break
    fi
done

if [ -e "$ARDUINO_PORT" ]; then
    echo ">>> sending neutral PWM ($NEUTRAL_PWM x $THRUSTER_COUNT) to $ARDUINO_PORT"
    ARDUINO_PORT="$ARDUINO_PORT" ARDUINO_BAUD="$ARDUINO_BAUD" \
    MSG="$(python3 -c "print(' '.join(['${NEUTRAL_PWM}']*${THRUSTER_COUNT}))")" python3 <<'PY'
import os, time, sys
try:
    import serial
except ImportError:
    sys.stderr.write("pyserial not available; cannot send kill command\n")
    sys.exit(1)

port = os.environ["ARDUINO_PORT"]
baud = int(os.environ["ARDUINO_BAUD"])
msg = os.environ["MSG"] + "\n"
try:
    with serial.Serial(port, baudrate=baud, timeout=1, exclusive=True) as ser:
        time.sleep(0.2)          # let the port settle after node released it
        for _ in range(10):      # repeat so the ESCs reliably latch neutral
            ser.write(msg.encode())
            ser.flush()
            time.sleep(0.05)
    print("Thrusters neutralized.")
except Exception as e:
    sys.stderr.write(f"Failed to send kill command: {e}\n")
    sys.exit(1)
PY
else
    echo ">>> WARNING: $ARDUINO_PORT not present; cannot neutralize thrusters." >&2
fi

# Let the heartbeat resume arming the Teensy watchdog now the port is free.
pkill -CONT -f "$HEARTBEAT_MATCH" 2>/dev/null && echo ">>> resumed heartbeat (SIGCONT)"

# --- 3. hard-kill the rest of the stack -------------------------------------
echo ">>> [3/3] Killing the rest of the stack..."
# One polite SIGINT to the launch leaders first, then a hard sweep of every
# node executable (they run out of the workspace's install/lib/<pkg>/).
pkill -INT -f "ros2 launch main" 2>/dev/null
sleep 1
for pat in \
    "ros2 launch main" \
    "ros2 bag record" \
    "install/lib/" \
    "robot_localization/lib" \
    "ekf_node" \
    "xsens_mti_node"; do
    pkill -KILL -f "$pat" 2>/dev/null
done

echo ">>> Done. (Heartbeat/systemd services are left running.)"
