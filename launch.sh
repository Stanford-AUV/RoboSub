#!/bin/bash

# Enable job control to manage background processes properly
set -m

# Directory for log files
LOG_DIR="/tmp/ros2_fzf_logs"
mkdir -p "$LOG_DIR"

# Declare ROS 2 nodes (or simulated processes)
declare -A NODES=(
    ["node1"]="while :; do echo 'Hello, one!'; sleep 1; done"
    ["node2"]="while :; do echo 'Hello, two!'; sleep 1; done"
    ["node3"]="while :; do echo 'Hello, three!'; sleep 1; done"
)

# Flag to ensure cleanup only runs once
CLEANUP_DONE=false

# Function to kill all child processes and clean up
cleanup() {
    if [ "$CLEANUP_DONE" = false ]; then
        CLEANUP_DONE=true
        echo "Stopping all nodes..."
        pkill -P $$  # Kill all background processes started by this script
        kill -- -$$  # Kill entire process group (ensures no orphaned processes)
        rm -rf "$LOG_DIR"  # Remove logs
        exit 0
    fi
}

# Trap signals to always clean up, ensuring it only runs once
trap cleanup SIGINT SIGTERM EXIT

# Start nodes in the background and log their output
for NODE in "${!NODES[@]}"; do
    eval "${NODES[$NODE]} > $LOG_DIR/$NODE.log 2>&1 &"
done

# Run fzf with nodes on the left and logs on the right
printf "%s\n" "${!NODES[@]}" | fzf --height=100% --layout=reverse --border \
    --preview "tail -f $LOG_DIR/{}\.log" --preview-window=right:90%:wrap

# Run cleanup when fzf exits
cleanup
