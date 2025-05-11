import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
from scipy.spatial.transform import Rotation as R
from hardware.utils.average_quaternions import average_quaternions

NO_IMU_POSITION = False
NO_IMU_ROTATION = False

NUM_CALIBRATION_QUATERNIONS = 200


class IMU(Node):
    def __init__(self):
        super().__init__("imu")
        self._imu_pub = self.create_publisher(Imu, "imu", 10)
        self.imu_subscription = self.create_subscription(
            Imu, "/imu/data", self.imu_listener_callback, 10
        )

        # Ideal 4×4 Transformation matrix (includes homogeneous coordinates)
        self.R_rot = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]])
        self.R_lin = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]])

        self.num_calibration_quaternions = 0
        self.calibration_quaternions = np.zeros((NUM_CALIBRATION_QUATERNIONS, 4))
        self.calibrated = False

    def imu_listener_callback(self, msg: Imu):
        if not self.calibrated:
            if self.num_calibration_quaternions == NUM_CALIBRATION_QUATERNIONS:
                self.calculate_transformation_matrix()
                self.calibrated = True
            else:
                q = np.array(
                    [
                        msg.orientation.x,
                        msg.orientation.y,
                        msg.orientation.z,
                        msg.orientation.w,
                    ]
                )
                transformed_q = self.transform_quaternion(q)
                self.calibration_quaternions[self.num_calibration_quaternions] = (
                    transformed_q
                )
                self.num_calibration_quaternions += 1
                return

        transformed_msg = self.transform_imu_msg(msg)
        transformed_msg.header.frame_id = "base_link"

        self._imu_pub.publish(transformed_msg)
        # self.get_logger().info(
        #     f"Sent IMU data: {transformed_msg}"
        # )

    def calculate_transformation_matrix(self):
        # Average quaternions
        avg_quat = average_quaternions(self.calibration_quaternions)
        # avg_quat = self.callibration_quaternions[0]
        r_imu = R.from_quat(avg_quat)  # convert to x, y, z, w
        # Robot is aligned with world for calibration, so its quaternion is identity
        r_robot = R.from_quat([0, 0, 0, 1])
        # Rotation from IMU to robot
        self.r_imu_to_robot = r_robot * r_imu.inv()
        r_matrix = self.r_imu_to_robot.as_matrix()
        self.get_logger().info(f"Calibrated with T:\n{r_matrix}")

    def transform_imu_msg(self, msg):
        transformed_msg = Imu()
        transformed_msg.header = msg.header

        # Transform orientation
        q = np.array(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        )
        transformed_q = (
            self.r_imu_to_robot * R.from_quat(self.transform_quaternion(q))
        ).as_quat()

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
        transformed_angular_velocity = self.R_rot @ angular_velocity
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
        transformed_linear_acceleration = self.R_lin @ linear_acceleration
        if NO_IMU_POSITION:
            transformed_msg.linear_acceleration.x = 0.0
            transformed_msg.linear_acceleration.y = 0.0
            transformed_msg.linear_acceleration.z = 0.0
        else:
            transformed_msg.linear_acceleration.x = transformed_linear_acceleration[0]
            transformed_msg.linear_acceleration.y = transformed_linear_acceleration[1]
            transformed_msg.linear_acceleration.z = transformed_linear_acceleration[2]

        return transformed_msg

    def transform_quaternion(self, q):
        q_matrix = R.from_quat(q).as_matrix()
        # Relabel axes: swap columns to say "Z is where Y used to be"
        # New X = Old Z
        # New Y = Old X
        # New Z = Old Y
        transformed_q_matrix = self.R_rot @ q_matrix @ self.R_rot.T
        transformed_q = R.from_matrix(transformed_q_matrix).as_quat()
        transformed_q /= np.linalg.norm(transformed_q)
        return transformed_q


def main(args=None):
    rclpy.init(args=args)
    imu = IMU()
    rclpy.spin(imu)
    imu.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
