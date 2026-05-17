#!/bin/bash
cd /workspaces/Robosub
source install/setup.bash

_term() {
    kill "$PID1" "$PID2" 2>/dev/null
    wait "$PID1" "$PID2" 2>/dev/null
}
trap _term TERM INT

ros2 launch src/main/launch/hardware.py &
PID1=$!
ros2 launch src/main/launch/localization.py &
PID2=$!