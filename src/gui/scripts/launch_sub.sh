#!/bin/bash
set -e
cd /workspaces/Robosub
source install/setup.bash
ros2 launch src/main/launch/hardware.py
