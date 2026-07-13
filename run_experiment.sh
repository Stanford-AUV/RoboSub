#!/usr/bin/env bash
#
# Minimal experiment bringup: EKF-filtered rotation live plot + thrust pipeline.
#
# Launches ONLY what the experiment needs, each in its own tmux pane:
#   xsens_mti_node -> hardware imu -> ekf_node        (rotation estimate)
#   thrust_generator -> thrusters -> arduino          (/wrench -> ESCs)
#   plot_rotation.py                                  (live graph)
#   rosbag record of the experiment topics            (ROSBAG=0 to skip)
#
# NOT launched: dvl, depth, sensors, control/PID, planning, perception.
# The EKF will warn about missing /position and /velocity inputs; that is
# expected — rotation comes from /imu/orientation alone.
#
# Usage:
#   ./run_experiment.sh          # bring everything up, attach to tmux
#   ./stop_experiment.sh         # tear down + neutralize thrusters
#
# Send thrust from the "cmd" tmux window, e.g.:
#   python send_thrust.py --fx 60 --duration 3
#
set -u

# Always target the DEFAULT tmux server. With $TMUX set (e.g. run from inside
# another tmux), every tmux call would silently go to a different socket and
# the session would seem to "vanish". Remember it only to decide on attaching.
WAS_IN_TMUX="${TMUX:-}"
unset TMUX

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESSION="cpu_imu_exp"
ROSBAG="${ROSBAG:-1}"
EKF_PARAMS="$REPO/src/localization/localization/nodes/ekf.yaml"

# Fast DDS async-publish profile so a dropped tether/topside subscriber can't
# block a reliable publish() and freeze a single-threaded node (e.g. the EKF).
# See config/fastdds_async.xml.
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-${REPO}/config/fastdds_async.xml}"

# Every pane gets the conda env + workspace overlay before its command. Also
# forward the DDS profile into each tmux pane (panes inherit the tmux SERVER env,
# which may predate this shell, so pass it explicitly).
ENV="export FASTRTPS_DEFAULT_PROFILES_FILE='$FASTRTPS_DEFAULT_PROFILES_FILE' && source ~/miniforge3/etc/profile.d/conda.sh && conda activate robosub && source '$REPO/install/setup.bash'"

if [ ! -f "$REPO/install/setup.bash" ]; then
    echo "ERROR: workspace not built ($REPO/install/setup.bash missing)." >&2
    exit 1
fi

# Re-assert yaml symlinks (colcon copies data_files yamls on every build, even
# with --symlink-install) so nodes never read stale install-space configs.
"$REPO/tools/symlink_yamls.sh" || {
    echo "ERROR: yaml symlinking failed - install space may serve STALE configs." >&2
    exit 1
}

tmux kill-session -t "$SESSION" 2>/dev/null

run_pane() {  # run_pane <target> <command...>
    local target="$1"; shift
    tmux send-keys -t "$target" "$ENV && $*" C-m
}

# --- window 0: sensing (xsens driver | imu republisher | ekf) ----------------
tmux new-session -d -s "$SESSION" -n sensing -c "$REPO"
tmux split-window -h -t "$SESSION:sensing" -c "$REPO"
tmux split-window -v -t "$SESSION:sensing.1" -c "$REPO"
run_pane "$SESSION:sensing.0" \
    'ros2 run xsens_mti_ros2_driver xsens_mti_node --ros-args --params-file "$(ros2 pkg prefix xsens_mti_ros2_driver)/share/xsens_mti_ros2_driver/param/xsens_mti_node.yaml"'
sleep 2
run_pane "$SESSION:sensing.1" \
    "ros2 run hardware imu"
sleep 1
run_pane "$SESSION:sensing.2" \
    "ros2 run robot_localization ekf_node --ros-args -r __node:=ekf_filter_node --params-file '$EKF_PARAMS'"

# --- window 1: thrust pipeline (thrust_generator | thrusters | arduino) ------
tmux new-window -t "$SESSION" -n thrust -c "$REPO"
tmux split-window -h -t "$SESSION:thrust" -c "$REPO"
tmux split-window -v -t "$SESSION:thrust.1" -c "$REPO"
run_pane "$SESSION:thrust.0" \
    "ros2 run control thrust_generator --log-level warn"
run_pane "$SESSION:thrust.1" \
    "ros2 run hardware thrusters --log-level warn"
sleep 1
run_pane "$SESSION:thrust.2" \
    "ros2 run hardware arduino"

# --- window 2: rosbag (experiment topics only) --------------------------------
if [ "$ROSBAG" = "1" ]; then
    BAG_DIR="$REPO/bags/imu_thrust_$(date +%Y%m%d_%H%M%S)"
    tmux new-window -t "$SESSION" -n bag -c "$REPO"
    run_pane "$SESSION:bag" \
        "mkdir -p '$REPO/bags' && ros2 bag record -o '$BAG_DIR' /wrench /thrusts /pwms /imu/orientation /filter/quaternion /odometry/filtered < /dev/null"
fi

# --- window 3: live rotation plot ---------------------------------------------
# tmux panes inherit the tmux SERVER's env, which may predate this X session;
# pin DISPLAY from the invoking shell so TkAgg finds the right X server.
tmux new-window -t "$SESSION" -n plot -c "$REPO"
run_pane "$SESSION:plot" \
    "export DISPLAY='${DISPLAY:-:0}' && python '$REPO/plot_rotation.py'"

# --- window 4: command shell for the user -------------------------------------
tmux new-window -t "$SESSION" -n cmd -c "$REPO"
run_pane "$SESSION:cmd" \
    "echo 'Send thrust, e.g.:  python send_thrust.py --fx 60 --duration 3   (see --help)'"

tmux select-window -t "$SESSION:cmd"
echo ">>> Experiment up in tmux session '$SESSION'."
echo ">>> Attach:   tmux attach -t $SESSION"
echo ">>> Teardown: $REPO/stop_experiment.sh"
# Attach unless already inside tmux or headless (NOATTACH=1).
if [ -z "$WAS_IN_TMUX" ] && [ "${NOATTACH:-0}" != "1" ]; then
    tmux attach -t "$SESSION"
fi
