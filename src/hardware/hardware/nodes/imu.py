import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
from scipy.spatial.transform import Rotation as R

NO_IMU_POSITION = False
NO_IMU_ROTATION = True


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
        self.T_rot = np.array([[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]])
        self.T_lin = np.array([[0, 0, 1, 0], [1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]])

        self._imu_pub = self.create_publisher(Imu, "imu", 10)

    def imu_listener_callback(self, msg):
        transformed_msg = self.transform_imu_msg(msg)
        # self.get_logger().info(
        #     f"Sending IMU data: {transformed_msg.linear_acceleration}"
        # )
        self._imu_pub.publish(transformed_msg)

    def transform_imu_msg(self, msg):
        transformed_msg = Imu()
        transformed_msg.header = msg.header

        # Transform orientation
        q = np.array(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        )
        q_matrix = quaternion_matrix(q)  # Get 4×4 rotation matrix
        transformed_q_matrix = (
            self.T_rot @ q_matrix @ self.T_rot.T
        )  # Apply transformation
        transformed_q = quaternion_from_matrix(transformed_q_matrix)

        # Normalize quaternion to avoid numerical drift
        transformed_q /= np.linalg.norm(transformed_q)

        if NO_IMU_ROTATION:
            transformed_msg.orientation.x = 0.0
            transformed_msg.orientation.y = 0.0
            transformed_msg.orientation.z = 0.0
            transformed_msg.orientation.w = 1.0
        else:
            transformed_msg.orientation.x = transformed_q[0]
            transformed_msg.orientation.y = transformed_q[1]
            transformed_msg.orientation.z = transformed_q[2]
            transformed_msg.orientation.w = transformed_q[3]

        # Transform angular velocity
        angular_velocity = np.array(
            [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        )
        transformed_angular_velocity = self.T_rot[:3, :3] @ angular_velocity
        if NO_IMU_ROTATION:
            transformed_msg.angular_velocity.x = 0.0
            transformed_msg.angular_velocity.y = 0.0
            transformed_msg.angular_velocity.z = 0.0
        else:
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
        transformed_linear_acceleration = self.T_lin[:3, :3] @ linear_acceleration
        if NO_IMU_POSITION:
            transformed_msg.linear_acceleration.x = 0.0
            transformed_msg.linear_acceleration.y = 0.0
            transformed_msg.linear_acceleration.z = 0.0
        else:
            transformed_msg.linear_acceleration.x = transformed_linear_acceleration[0]
            transformed_msg.linear_acceleration.y = transformed_linear_acceleration[1]
            transformed_msg.linear_acceleration.z = transformed_linear_acceleration[2]

        return transformed_msg


def main(args=None):
    rclpy.init(args=args)
    imu = IMU()
    rclpy.spin(imu)
    imu.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
