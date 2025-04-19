#!/usr/bin/env python3
import sys
import rclpy
from rclpy.serialization import serialize_message, deserialize_message
from rosbag2_py import SequentialReader, SequentialWriter, StorageOptions, ConverterOptions

def sync_bag(input_bag_path, output_bag_path):
    # Configure storage options for input and output bags (using mcap backend)
    storage_options_in = StorageOptions(uri=input_bag_path, storage_id="mcap")
    storage_options_out = StorageOptions(uri=output_bag_path, storage_id="mcap")
    converter_options = ConverterOptions(input_serialization_format='cdr',
                                          output_serialization_format='cdr')
    
    # Open the input bag for reading.
    reader = SequentialReader()
    reader.open(storage_options_in, converter_options)
    
    # Retrieve topic metadata so we can re-create topics in the output bag.
    topics_meta = reader.get_all_topics_and_types()

    # Open the writer for the output bag.
    writer = SequentialWriter()
    writer.open(storage_options_out, converter_options)
    for meta in topics_meta:
        writer.create_topic(meta)
    
    # Initialize variables to store the first DVL and IMU timestamps (in seconds)
    # and the computed offset.
    first_dvl_time = None
    first_imu_time = None
    imu_offset = None

    # Process each message in the input bag.
    while reader.has_next():
        topic, data, timestamp = reader.read_next()  # timestamp is in nanoseconds

        if topic == "/imu":
            # Import the IMU message type.
            from sensor_msgs.msg import Imu
            msg = deserialize_message(data, Imu)
            # Record the first IMU timestamp.
            if first_imu_time is None:
                first_imu_time = timestamp / 1e9  # convert nanoseconds to seconds
                if first_dvl_time is not None:
                    imu_offset = first_dvl_time - first_imu_time
                    print(f"Computed IMU offset: {imu_offset:.6f} sec")
            # If the offset is known, adjust the timestamp.
            if imu_offset is not None:
                new_time_sec = (timestamp / 1e9) + imu_offset
                new_timestamp = int(new_time_sec * 1e9)
                # Update the message header stamp.
                msg.header.stamp.sec = new_timestamp // 1000000000
                msg.header.stamp.nanosec = new_timestamp % 1000000000
                new_data = serialize_message(msg)
                writer.write(topic, new_data, new_timestamp)
            else:
                # If offset not computed yet, write the message as-is.
                writer.write(topic, data, timestamp)

        elif topic == "/dvl":
            # Import the DVL message type.
            from msgs.msg import DVLData
            msg = deserialize_message(data, DVLData)
            # Record the first DVL timestamp (assumed to be correct epoch time).
            if first_dvl_time is None:
                first_dvl_time = timestamp / 1e9  # convert nanoseconds to seconds
                if first_imu_time is not None:
                    imu_offset = first_dvl_time - first_imu_time
                    print(f"Computed IMU offset: {imu_offset:.6f} sec")
            # Write the DVL message unmodified.
            writer.write(topic, data, timestamp)

        else:
            # For any other topics, write the message unchanged.
            writer.write(topic, data, timestamp)

def main():
    if len(sys.argv) < 3:
        print("Usage: sync_bag_ros2.py <input_bag_directory> <output_bag_directory>")
        sys.exit(1)
    input_bag = sys.argv[1]
    output_bag = sys.argv[2]
    rclpy.init(args=sys.argv)
    sync_bag(input_bag, output_bag)
    rclpy.shutdown()
    print("Bag sync complete.")

if __name__ == '__main__':
    main()
