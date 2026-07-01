#!/usr/bin/env bash
#
# Bring up the full stack (hardware -> localization -> control -> planning),
# then on Ctrl+C shut everything down and send neutral PWM (1500) to every
# thruster so the sub powers down safely.
#
# Usage:  ./launch_sub.sh
#
set -u

# --- config -----------------------------------------------------------------
ROS_DISTRO_SETUP="/opt/ros/jazzy/setup.bash"
ARDUINO_PORT="/dev/ttyACM0"
ARDUINO_BAUD=9600
THRUSTER_COUNT=8
NEUTRAL_PWM=1500

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_SETUP="${REPO_DIR}/install/setup.bash"

LAUNCHES=(hardware localization control planning)
PIDS=()
KILLED=0

# --- source ROS -------------------------------------------------------------
# ROS/ament setup scripts reference unbound vars (e.g. AMENT_TRACE_SETUP_FILES),
# so temporarily disable nounset while sourcing them.
set +u
# shellcheck disable=SC1090
[ -f "$ROS_DISTRO_SETUP" ] && source "$ROS_DISTRO_SETUP"
if [ -f "$WS_SETUP" ]; then
    # shellcheck disable=SC1090
    source "$WS_SETUP"
else
    set -u
    echo "ERROR: workspace not built ($WS_SETUP missing). Run 'colcon build' first." >&2
    exit 1
fi
set -u

# --- neutralize thrusters ---------------------------------------------------
# Sends a string of 1500s (one per thruster) straight to the Arduino serial
# port, matching the format the arduino node uses ("1500 1500 ...\n").
kill_thrusters() {
    local msg
    msg="$(python3 -c "print(' '.join(['${NEUTRAL_PWM}']*${THRUSTER_COUNT}))")"

    # Wait for the arduino node to release the port (it holds it exclusively).
    for _ in $(seq 1 20); do
        if command -v fuser >/dev/null 2>&1 && fuser -s "$ARDUINO_PORT" 2>/dev/null; then
            sleep 0.1
        else
            break
        fi
    done

    if [ ! -e "$ARDUINO_PORT" ]; then
        echo "WARNING: $ARDUINO_PORT not present; cannot send kill command." >&2
        return 1
    fi

    echo ">>> Sending neutral PWM ($NEUTRAL_PWM x $THRUSTER_COUNT) to $ARDUINO_PORT"
    ARDUINO_PORT="$ARDUINO_PORT" ARDUINO_BAUD="$ARDUINO_BAUD" MSG="$msg" python3 <<'PY'
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
    with serial.Serial(port, baudrate=baud, timeout=1) as ser:
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
}

# --- shutdown handler -------------------------------------------------------
shutdown() {
    [ "$KILLED" -eq 1 ] && return
    KILLED=1
    echo
    echo ">>> Caught Ctrl+C - shutting down stack..."

    # Gracefully stop each ros2 launch (SIGINT lets it tear down its children).
    for pid in "${PIDS[@]}"; do
        kill -INT "$pid" 2>/dev/null
    done
    for pid in "${PIDS[@]}"; do
        wait "$pid" 2>/dev/null
    done

    kill_thrusters
    echo ">>> Done."
    exit 0
}
trap shutdown INT TERM

# --- launch -----------------------------------------------------------------
echo ">>> Launching stack: ${LAUNCHES[*]}"
for lf in "${LAUNCHES[@]}"; do
    echo ">>> ros2 launch main ${lf}.py"
    ros2 launch main "${lf}.py" &
    PIDS+=("$!")
    sleep 2   # stagger so hardware/localization come up before control/planning
done

echo ">>> Stack running (PIDs: ${PIDS[*]}). Press Ctrl+C to stop and kill thrusters."

# Wait; if any launch dies on its own, tear the rest down too.
wait -n 2>/dev/null || true
shutdown
