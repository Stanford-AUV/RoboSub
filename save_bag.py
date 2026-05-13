#!/usr/bin/env python3
import os
import cv2
import sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

def bag_to_mp4(bag_path, topic_name, output_path, fps=30):
    bridge = CvBridge()
    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path,
        storage_id='mcap' if bag_path.endswith('.mcap') else 'sqlite3'
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # Get topic type info
    topics = reader.get_all_topics_and_types()
    topic_type_map = {t.name: t.type for t in topics}

    if topic_name not in topic_type_map:
        print(f"Topic {topic_name} not found in bag!")
        return

    # --- Count total frames for progress ---
    total_frames = 0
    tmp_reader = rosbag2_py.SequentialReader()
    tmp_reader.open(storage_options, converter_options)
    while tmp_reader.has_next():
        topic, _, _ = tmp_reader.read_next()
        if topic == topic_name:
            total_frames += 1
    del tmp_reader

    if total_frames == 0:
        print(f"No messages found for topic {topic_name}")
        return

    # Prepare output video
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video_writer = None
    frame_size = None

    processed_frames = 0
    last_progress = -1  # last percentage reported

    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == topic_name:
            msg = deserialize_message(data, CompressedImage)
            frame = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")

            if video_writer is None:
                frame_size = (frame.shape[1], frame.shape[0])
                video_writer = cv2.VideoWriter(output_path, fourcc, fps, frame_size)

            video_writer.write(frame)

            processed_frames += 1
            progress = int((processed_frames / total_frames) * 100)
            if progress % 5 == 0 and progress != last_progress:
                print(f"Progress: {progress}% ({processed_frames}/{total_frames})")
                last_progress = progress

    if video_writer:
        video_writer.release()
        print(f"Saved video to {output_path}")
    else:
        print("No frames were written!")

if __name__ == "__main__":
    if len(sys.argv) != 4:
        print(f"Usage: {sys.argv[0]} <bag_path> <topic_name> <output_mp4>")
        sys.exit(1)

    bag_to_mp4(sys.argv[1], sys.argv[2], sys.argv[3])
