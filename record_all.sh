#!/bin/bash

echo "Make sure all launch files are already running before running this script. This includes camera, localization, hardware, etc."

# Check if output path was provided
if [ -z "$1" ]; then
  echo "Usage: $0 OUTPUT_PATH"
  exit 1
fi

OUTPUT_PATH="$1"

# Get all topics under /forward_cam/ and /bottom_cam/
TOPICS=$(ros2 topic list)

if [ -z "$TOPICS" ]; then
  echo "No matching topics found"
  exit 1
fi

# Record bag with dynamically gathered topics
ros2 bag record -o "$OUTPUT_PATH" $TOPICS