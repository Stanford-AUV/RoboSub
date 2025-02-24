# Get a timestamp for the output folder.
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
OUTPUT_FOLDER="/replays/${TIMESTAMP}"

# Create the output folder.
mkdir -p "$OUTPUT_FOLDER"

echo "Recording topics: dvl imu pressure temperature depth current voltage"
echo "Bag files will be stored in: $OUTPUT_FOLDER"

ros2 bag record -o "$OUTPUT_FOLDER" dvl imu pressure temperature depth current voltage
