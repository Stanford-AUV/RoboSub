#!/bin/bash

echo "Make sure camera launch file is fully loaded and both cameras have printed \"Camera ready!\" before running this script."

# Check if output path was provided
if [ -z "$1" ]; then
  echo "Usage: $0 OUTPUT_PATH"
  exit 1
fi

OUTPUT_PATH="$1"

# Get all topics under /forward_cam/ and /bottom_cam/
TOPICS=$(ros2 topic list | grep -E '^/forward_cam/|^/bottom_cam/')

if [ -z "$TOPICS" ]; then
  echo "No matching topics found under /forward_cam/ or /bottom_cam/"
  exit 1
fi

# Record bag with dynamically gathered topics
ros2 bag record -o "$OUTPUT_PATH" $TOPICS