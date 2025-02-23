#!/bin/bash

# Enable job control to manage background processes properly
set -m

# Log file containing all ROS 2 logs
LOG_FILE="$HOME/.ros/log/latest/launch.log"

# Function to get active ROS 2 nodes dynamically
get_nodes() {
    ros2 node list 2>/dev/null | sed 's/^\///' | sort | uniq  # Remove leading '/'
}

# Run fzf with dynamic ROS 2 node selection and correctly filtered logs
get_nodes | fzf --height=100% --layout=reverse --border \
    --bind "change:reload(get_nodes)" \
    --preview "grep --line-buffered '{}' '$LOG_FILE' || echo 'No logs for {} yet...'" \
    --preview-window=right:80%:wrap
