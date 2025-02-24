import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
from scipy.spatial.transform import Rotation as R


def quaternion_matrix(quaternion):
    """Convert quaternion to a homogeneous 4×4 rotation matrix."""
    q = np.array(quaternion, dtype=np.float64)
    if np.dot(q, q) < np.finfo(float).eps * 4.0:  # Check for zero quaternion
        return np.identity(4)

    r = R.from_quat(q)
    M = np.eye(4)
    M[:3, :3] = r.as_matrix()
    return M


def quaternion_from_matrix(matrix):
    """Extract a unit quaternion from a 4×4 rotation matrix."""
    return R.from_matrix(matrix[:3, :3]).as_quat()


class IMU(Node):
    def __init__(self):
        super().__init__("imu")
        self.imu_subscription = self.create_subscription(
            Imu, "/imu/data", self.imu_listener_callback, 10
        )

        # 4×4 Transformation matrix (includes homogeneous coordinates)
        self.T = np.array([[0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

        self._imu_pub = self.create_publisher(Imu, "imu", 10)

    def imu_listener_callback(self, msg):
        transformed_msg = self.transform_imu_msg(msg)
        self.get_logger().info(
            f"IMU data: {transformed_msg.header}"
        )
        self._imu_pub.publish(transformed_msg)

    def transform_imu_msg(self, msg):
        transformed_msg = Imu()
        transformed_msg.header = msg.header

        # Transform orientation
        q = np.array(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        )
        q_matrix = quaternion_matrix(q)  # Get 4×4 rotation matrix
        transformed_q_matrix = self.T @ q_matrix @ self.T.T  # Apply transformation
        transformed_q = quaternion_from_matrix(transformed_q_matrix)

        # Normalize quaternion to avoid numerical drift
        transformed_q /= np.linalg.norm(transformed_q)

        transformed_msg.orientation.x = transformed_q[0]
        transformed_msg.orientation.y = transformed_q[1]
        transformed_msg.orientation.z = transformed_q[2]
        transformed_msg.orientation.w = transformed_q[3]

        # Transform angular velocity
        angular_velocity = np.array(
            [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        )
        transformed_angular_velocity = self.T[:3, :3] @ angular_velocity
        transformed_msg.angular_velocity.x = transformed_angular_velocity[0]
        transformed_msg.angular_velocity.y = transformed_angular_velocity[1]
        transformed_msg.angular_velocity.z = transformed_angular_velocity[2]

        # Transform linear acceleration
        linear_acceleration = np.array(
            [
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            ]
        )
        transformed_linear_acceleration = self.T[:3, :3] @ linear_acceleration
        transformed_msg.linear_acceleration.x = transformed_linear_acceleration[0]
        transformed_msg.linear_acceleration.y = transformed_linear_acceleration[1]
        transformed_msg.linear_acceleration.z = transformed_linear_acceleration[2]

        # print("hi")

        return transformed_msg


def main(args=None):
    rclpy.init(args=args)
    imu = IMU()
    rclpy.spin(imu)
    imu.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
