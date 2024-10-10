#!/bin/bash

# Function to handle cleanup
cleanup() {
    echo "Cleaning up..."
    # Kill all background processes
    pkill -P $$
    exit 1
}

# Trap SIGINT (Ctrl+C)
trap cleanup SIGINT

# Set environment variables
export GZ_PARTITION=127.0.0.1:ros
export DISPLAY=:0
export GZ_SIM_RESOURCE_PATH=`pwd`/src/simulation/simulation/models:$GZ_SIM_RESOURCE_PATH
export GZ_SIM_SYSTEM_PLUGIN_PATH=`pwd`/src/simulation/simulation/models/CartPole/plugins:$GZ_SIM_SYSTEM_PLUGIN_PATH 

# Start commands in the background
gz sim src/simulation/simulation/models/world.urdf &
SIM_PID=$!

ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=src/simulation/simulation/gazebo_bridge.yaml &
BRIDGE_PID=$!

ros2 launch src/launch/simulation.py &
LAUNCH_PID=$!

# Wait for all background processes to finish
wait $SIM_PID $BRIDGE_PID $LAUNCH_PID
