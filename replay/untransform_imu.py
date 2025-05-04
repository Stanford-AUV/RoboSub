import rclpy
from sensor_msgs.msg import Imu
from rosbag2_py import SequentialReader, SequentialWriter, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message, serialize_message
import numpy as np
from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_matrix, quaternion_from_matrix
from rosbag2_py._storage import TopicMetadata

INPUT_BAG = "0419/0419_0.mcap"
OUTPUT_BAG = "0419_new"

# Define your 4x4 transformation matrix (currently identity)
MATRIX = np.array([
    [0, 0, -1, 0],
    [-1, 0, 0, 0],
    [0, -1, 0, 0],
    [0, 0, 0, 1]
])

def transform_imu(imu_msg: Imu) -> Imu:
    T = MATRIX

    # --- Orientation ---
    q = imu_msg.orientation
    quat = [q.x, q.y, q.z, q.w]
    R_mat = quaternion_matrix(quat)
    R_new = T.T @ R_mat @ T
    q_new = quaternion_from_matrix(R_new)
    q_new /= np.linalg.norm(q_new)

    imu_msg.orientation = Quaternion(x=q_new[0], y=q_new[1], z=q_new[2], w=q_new[3])

    # --- Angular Velocity ---
    w = np.array([
        imu_msg.angular_velocity.x,
        imu_msg.angular_velocity.y,
        imu_msg.angular_velocity.z
    ])
    w_new = T[:3, :3].T @ w
    imu_msg.angular_velocity.x = w_new[0]
    imu_msg.angular_velocity.y = w_new[1]
    imu_msg.angular_velocity.z = w_new[2]

    # --- Linear Acceleration ---
    a = np.array([
        imu_msg.linear_acceleration.x,
        imu_msg.linear_acceleration.y,
        imu_msg.linear_acceleration.z
    ])
    a_new = T[:3, :3].T @ a
    imu_msg.linear_acceleration.x = a_new[0]
    imu_msg.linear_acceleration.y = a_new[1]
    imu_msg.linear_acceleration.z = a_new[2]

    return imu_msg


def main():
    rclpy.init()

    # Reader
    reader = SequentialReader()
    reader.open(StorageOptions(uri=INPUT_BAG, storage_id='mcap'),
                ConverterOptions('cdr', 'cdr'))

    # Writer
    writer = SequentialWriter()
    writer.open(StorageOptions(uri=OUTPUT_BAG, storage_id='mcap'),
                ConverterOptions('cdr', 'cdr'))

    topic_types = reader.get_all_topics_and_types()

    for topic in topic_types:
        if topic.name == '/imu':
            # Create only /imu/data
            writer.create_topic(TopicMetadata(
                0, '/imu/data', topic.type, 'cdr'
            ))
        else:
            writer.create_topic(TopicMetadata(
                0, topic.name, topic.type, topic.serialization_format
            ))

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic == '/imu':
            imu_msg = deserialize_message(data, Imu)
            transformed = transform_imu(imu_msg)
            writer.write('/imu/data', serialize_message(transformed), timestamp)
        else:
            writer.write(topic, data, timestamp)

    print(f"✅ Saved transformed bag to {OUTPUT_BAG}")

if __name__ == "__main__":
    main()
