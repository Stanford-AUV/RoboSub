#!/usr/bin/env bash
#
# Bring up the full stack (imu/xsens -> hardware -> localization -> control ->
# planning), then on Ctrl+C shut everything down and send neutral PWM (1500) to
# every thruster so the sub powers down safely.
#
# Usage:  conda activate robosub && ./launch_sub.sh
#
set -u

# --- config -----------------------------------------------------------------
# ROS 2 is provided by the active conda env (`conda activate robosub`), so no
# system setup file is sourced by default. For a native apt ROS install instead,
# run with ROS_DISTRO_SETUP=/opt/ros/<distro>/setup.bash.
ROS_DISTRO_SETUP="${ROS_DISTRO_SETUP:-}"
ARDUINO_PORT="/dev/ttyACM0"
ARDUINO_BAUD=9600
THRUSTER_COUNT=8
NEUTRAL_PWM=1500

# The Orin->Teensy heartbeat (systemd: orin-heartbeat.service, runs as this
# user) shares $ARDUINO_PORT with the arduino node and re-grabs it the instant
# that node exits. During shutdown we freeze it with SIGSTOP so it can't
# interleave 'heartbeat N' bytes with our neutral-PWM burst, then SIGCONT it.
# Same-user signals need no sudo, and a stopped (not dead) process is NOT
# respawned by systemd's Restart=always.
HEARTBEAT_MATCH="orin_heartbeat.py"

# Record everything to a rosbag so runs are reviewable. Set ROSBAG=0 to skip.
ROSBAG="${ROSBAG:-1}"

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_SETUP="${REPO_DIR}/install/setup.bash"
BAG_ROOT="${REPO_DIR}/bags"

# Fast DDS async-publish profile. Without this, the default RELIABLE+SYNCHRONOUS
# writers block inside publish() when a topside subscriber over the tether
# vanishes (untethered runs) -> the single-threaded EKF froze for up to 36s.
# ASYNCHRONOUS publish + a 10ms max_blocking_time make publish() non-blocking so
# a dropped tether/peer can never stall an executor. See config/fastdds_async.xml.
# Override by exporting FASTRTPS_DEFAULT_PROFILES_FILE before running.
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-${REPO_DIR}/config/fastdds_async.xml}"

# Three-phase bringup:
#   1. IMU + Xsens driver ONLY, then wait IMU_SETTLE_DELAY so the Xsens onboard
#      filter (AHS heading + orientation) converges before anything consumes it.
#   2. The rest of the base stack (hardware I/O, localization, control), then
#      wait PATH_DELAY so the EKF converges and thrusters idle at neutral.
#   3. The path / planning nodes that issue setpoints.
LAUNCHES_IMU=(imu)
LAUNCHES_BASE=(hardware localization control)
LAUNCHES_PATH=(planning)
IMU_SETTLE_DELAY="${IMU_SETTLE_DELAY:-20}"
PATH_DELAY="${PATH_DELAY:-5}"
PIDS=()
HW_PID=""     # process group of the hardware launch (set once it starts)
BAG_PID=""    # process group of the rosbag recorder (set once it starts)
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
    # exclusive=True (TIOCEXCL) so nothing else can re-open the port mid-burst.
    # The heartbeat is SIGSTOP-frozen by the caller for the same reason; this is
    # belt-and-suspenders against any other opener.
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
}

# --- shutdown handler -------------------------------------------------------
# Poll until every process in group $1 is gone, or $2 seconds elapse.
# Returns 0 if the group emptied, 1 on timeout. A negative PID targets the
# whole process group, so this still detects nodes after their launch leader
# has exited (the group id outlives the leader).
wait_group_gone() {
    local pgid="$1" deadline=$(( SECONDS + $2 ))
    while [ "$SECONDS" -lt "$deadline" ]; do
        kill -0 "-$pgid" 2>/dev/null || return 0
        sleep 0.2
    done
    kill -0 "-$pgid" 2>/dev/null && return 1
    return 0
}

# Tear down a single launch's process GROUP (negative PID): one clean SIGINT,
# then escalate to SIGTERM/SIGKILL for anything that ignores it.
kill_group() {
    local pid="$1"
    kill -INT "-$pid" 2>/dev/null
    if ! wait_group_gone "$pid" 8; then
        echo ">>> group $pid ignored SIGINT; escalating to SIGTERM/SIGKILL"
        kill -TERM "-$pid" 2>/dev/null
        wait_group_gone "$pid" 3 || kill -KILL "-$pid" 2>/dev/null
    fi
}

shutdown() {
    [ "$KILLED" -eq 1 ] && return
    KILLED=1
    echo
    echo ">>> Caught Ctrl+C - killing thrusters first..."

    # THRUSTERS FIRST: the hardware node (arduino) holds the serial port
    # exclusively, so we must take it down before we can drive neutral PWM onto
    # the port. Kill ONLY the hardware group, then immediately neutralize the
    # thrusters — this is the fastest path to a safe, non-spinning sub. The rest
    # of the stack (rosbag/localization/control/planning) is torn down after.
    #
    # Each launch runs in its OWN process group ('set -m'), so we signal the
    # whole GROUP (negative PID) to reach every node it spawned.
    #
    # Freeze the heartbeat FIRST so it can't grab $ARDUINO_PORT the moment the
    # arduino node dies and corrupt our neutral burst. No-op if it isn't running.
    pkill -STOP -f "$HEARTBEAT_MATCH" 2>/dev/null && echo ">>> Froze heartbeat (SIGSTOP)"
    if [ "${HW_PID:-}" ]; then
        kill_group "$HW_PID"
    fi
    kill_thrusters
    # Let the heartbeat resume arming the Teensy watchdog now the port is free.
    pkill -CONT -f "$HEARTBEAT_MATCH" 2>/dev/null && echo ">>> Resumed heartbeat (SIGCONT)"

    # Close the rosbag cleanly (SIGINT lets rosbag2 flush and write metadata.yaml;
    # a SIGKILL here would leave the bag unindexed/unreadable).
    if [ "${BAG_PID:-}" ]; then
        echo ">>> Stopping rosbag..."
        kill -INT "-$BAG_PID" 2>/dev/null
        wait_group_gone "$BAG_PID" 10 || kill -KILL "-$BAG_PID" 2>/dev/null
    fi

    # Now tear down everything else (skip the already-handled hardware/bag groups).
    # (Previously localization/EKF survived Ctrl+C and kept publishing a stale
    # /odometry/filtered pose, which the next run's PID then chased the wrong way.)
    echo ">>> Shutting down the rest of the stack..."
    for pid in "${PIDS[@]}"; do
        [ "$pid" = "${HW_PID:-}" ] && continue
        [ "$pid" = "${BAG_PID:-}" ] && continue
        kill_group "$pid"
    done

    # Final hard sweep: process-group kills above miss a node if it re-parented
    # or landed outside its launch's group (this is what left duplicate EKFs
    # running before). Pattern-kill any stray stack executable by name so a
    # single Ctrl+C is guaranteed to leave no survivors. Bag already handled
    # cleanly above, so it is safe to SIGKILL anything matching here.
    for pat in \
        "ros2 launch main" \
        "install/lib/" \
        "robot_localization/lib" \
        "ekf_node" \
        "xsens_mti_node"; do
        pkill -KILL -f "$pat" 2>/dev/null
    done

    echo ">>> Done."
    exit 0
}
trap shutdown INT TERM

# --- launch -----------------------------------------------------------------
# Enable job control so each 'ros2 launch' lands in its OWN process group.
# Two payoffs: (1) the terminal's Ctrl+C hits only THIS script (the foreground
# group), not the launches directly, so ros2 launch gets exactly one clean
# SIGINT from our trap instead of a racy double-signal that makes it abort
# teardown and orphan nodes; (2) shutdown() can kill each launch's entire group
# to guarantee no survivors (this is what left localization running before).
set -m

# Launch one group of ros2 launch files, staggering each by 2s and recording
# its PID (== process group id, thanks to 'set -m') for shutdown.
launch_group() {
    for lf in "$@"; do
        echo ">>> ros2 launch main ${lf}.py"
        ros2 launch main "${lf}.py" &
        local pid="$!"
        PIDS+=("$pid")
        # Remember the hardware launch's group so shutdown() can take it down
        # first and neutralize the thrusters before anything else.
        [ "$lf" = "hardware" ] && HW_PID="$pid"
        sleep 2   # stagger so nodes come up cleanly one at a time
    done
}

# Break each settle-sleep into 1s ticks so a Ctrl+C during the delay is caught
# promptly (the trap fires between ticks) instead of after the full delay.
sleep_interruptible() {
    for _ in $(seq 1 "$1"); do
        [ "$KILLED" -eq 1 ] && break
        sleep 1
    done
}

# --- Phase 1: IMU + Xsens driver only ---------------------------------------
echo ">>> Phase 1 - IMU + Xsens: ${LAUNCHES_IMU[*]}"
launch_group "${LAUNCHES_IMU[@]}"

# Start recording as early as possible so the Xsens settle period is captured.
# 'ros2 bag record -a' keeps discovering topics that appear later, so the
# hardware/localization/control/planning nodes are captured too.
if [ "$ROSBAG" = "1" ] && [ "$KILLED" -eq 0 ]; then
    BAG_DIR="${BAG_ROOT}/run_$(date +%Y%m%d_%H%M%S)"
    mkdir -p "$BAG_ROOT"
    echo ">>> Recording rosbag -> $BAG_DIR"
    # stdin MUST be detached from the terminal (</dev/null): ros2 bag record has
    # interactive keyboard handling (SPACE to pause) that calls tcsetattr() on the
    # controlling tty. As a background job under job control (set -m), touching the
    # tty raises SIGTTOU/SIGTTIN and the recorder is stopped/killed BEFORE it opens
    # the bag - silently, so you get an empty bags/ and no error. Detaching stdin
    # makes it disable keyboard handling and record headless.
    ros2 bag record -a --include-hidden-topics -o "$BAG_DIR" </dev/null &
    BAG_PID="$!"
fi

# Let the Xsens onboard filter converge before the rest of the stack consumes it.
echo ">>> IMU up. Waiting ${IMU_SETTLE_DELAY}s for the Xsens filter to settle before starting: ${LAUNCHES_BASE[*]}"
sleep_interruptible "$IMU_SETTLE_DELAY"

# --- Phase 2: rest of the base stack ----------------------------------------
if [ "$KILLED" -eq 0 ]; then
    echo ">>> Phase 2 - base stack: ${LAUNCHES_BASE[*]}"
    launch_group "${LAUNCHES_BASE[@]}"
fi

# Let the EKF converge and the thrusters idle at neutral before planning starts
# issuing setpoints.
echo ">>> Base up. Waiting ${PATH_DELAY}s before starting: ${LAUNCHES_PATH[*]}"
sleep_interruptible "$PATH_DELAY"

# --- Phase 3: path / planning -----------------------------------------------
if [ "$KILLED" -eq 0 ]; then
    echo ">>> Phase 3 - path: ${LAUNCHES_PATH[*]}"
    launch_group "${LAUNCHES_PATH[@]}"
fi

echo ">>> Stack running (PIDs: ${PIDS[*]}). Press Ctrl+C to stop and kill thrusters."

# Wait; if any launch dies on its own, tear the rest down too.
wait -n 2>/dev/null || true
shutdown
