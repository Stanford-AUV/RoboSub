#!/bin/bash

echo "Make sure camera launch file is fully loaded and both cameras have printed \"Camera ready!\" before running this script."

# Check if output path was provided
if [ -z "$1" ]; then
  echo "Usage: $0 OUTPUT_PATH"
  exit 1
fi

OUTPUT_PATH="$1"

TOPICS="/wrench
/sensors
/depth
/dvl
/imu
/imu/data
/dvl/twist_sync
/imu/data_sync
/imu/pose_sync
/light
/odometry/filtered
/forward_cam/rgb/image_rect
/forward_cam/rgb/image_rect/compressed
/forward_cam/stereo/image_raw
/forward_cam/stereo/image_raw/compressed
/bottom_cam/rgb/image_rect
/bottom_cam/rgb/image_rect/compressed
/bottom_cam/stereo/image_raw
/bottom_cam/stereo/image_raw/compressed"

# Record bag with dynamically gathered topics
ros2 bag record -o "$OUTPUT_PATH" $TOPICS