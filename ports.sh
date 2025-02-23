#!/bin/bash

# List of possible device paths
DEVICES=(
    "/dev/ttyUSB_teensy"
    "/dev/ttyUSB0" "/dev/ttyUSB1" "/dev/ttyUSB2" "/dev/ttyUSB3" "/dev/ttyUSB4" "/dev/ttyUSB5"
    "/dev/ttyACM0" "/dev/ttyACM1" "/dev/ttyACM2" "/dev/ttyACM3" "/dev/ttyACM4" "/dev/ttyACM5"
)

# Loop through each device and apply chmod if it exists
for DEVICE in "${DEVICES[@]}"; do
    if [[ -e "$DEVICE" ]]; then
        chmod a+rw "$DEVICE"
        echo "Permissions set for $DEVICE"
    else
        echo "Skipping $DEVICE (not found)"
    fi
done