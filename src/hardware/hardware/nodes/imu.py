import rclpy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
import numpy as np
from hardware.nodes.generic_sensor import GenericSensor

from scipy.spatial.transform import Rotation as R

GYRO_BIAS = np.array(
    [
        -0.006099891562248375,
        -0.0034746338098969654,
        0.00273246756079096,
    ],
    dtype=float,
)

_BIG = 1e9


class IMU(GenericSensor):
    def __init__(self):
        super().__init__("imu", "imu_0")

        self.is_initialized = False
        self.q_init_inv = None

        if self.is_active("rotation"):
            self.sensor_publishers["rotation"] = self.create_publisher(
                PoseWithCovarianceStamped, "/rotation", 10
            )
        if self.is_active("angular"):
            self.sensor_publishers["angular"] = self.create_publisher(
                TwistWithCovarianceStamped, "/angular", 10
            )
        if self.is_active("accel"):
            self.sensor_publishers["accel"] = self.create_publisher(Imu, "/accel", 10)

        self.imu_subscription = self.create_subscription(
            Imu, "/imu/data", self._imu_callback, 10
        )

        self._latest_msg = None

    def _imu_callback(self, msg: Imu):
        w = np.array(
            [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            dtype=float,
        )
        w -= GYRO_BIAS
        w_base = self.R_sensor_to_base @ w
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = (
            w_base.tolist()
        )

        msg.header.frame_id = "base_link"

        q_raw = np.array(
            [
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w,
            ],
            dtype=float,
        )
        if (not np.all(np.isfinite(q_raw))) or (np.linalg.norm(q_raw) < 1e-6):
            self.get_logger().warn("Invalid IMU quaternion; dropping msg")
            return

        q_raw /= np.linalg.norm(q_raw)

        rot_sensor = R.from_quat(q_raw)
        rot_transform = R.from_matrix(self.R_sensor_to_base)

        rot_base = rot_transform * rot_sensor

        qx, qy, qz, qw = rot_base.as_quat()
        qw, qx, qy, qz = self.rotate_quaternion(qw, qx, qy, qz)

        rot_final = R.from_quat([qx, qy, qz, qw])

        # 1. Convert the final transformed quaternion to Euler angles (in radians)
        # Using 'xyz' ensures standard Roll (x), Pitch (y), and Yaw (z) mapping
        r_current, p_current, y_current = rot_final.as_euler("xyz")

        if not self.is_initialized:
            # Capture the exact orientation errors on the very first frame
            self.roll_offset = r_current
            self.pitch_offset = p_current
            self.yaw_offset = y_current

            self.is_initialized = True
            self.get_logger().info(
                "IMU 3D Orientation successfully zeroed via Euler mapping!"
            )

        # 2. Subtract the initial offsets directly to isolate relative movement
        r_zeroed = r_current - self.roll_offset
        p_zeroed = p_current - self.pitch_offset
        y_zeroed = y_current - self.yaw_offset

        # 3. Convert the zeroed Euler angles back into a pure unit quaternion
        rot_zeroed = R.from_euler("xyz", [r_zeroed, p_zeroed, y_zeroed])
        qx, qy, qz, qw = rot_zeroed.as_quat()

        msg.orientation.x = float(qx)
        msg.orientation.y = float(qy)
        msg.orientation.z = float(qz)
        msg.orientation.w = float(qw)

        self._latest_msg = msg
        self.publish_sensor_data()

    def _build_rotation_cov(self):
        """6x6 pose covariance: large values for position, yaml values for orientation."""
        cov = np.zeros(36)
        rot_axes = self.get_axes("rotation")
        rot_cov = self.get_covariance("rotation")

        for i in range(3):
            cov[i * 6 + i] = _BIG

        if rot_cov is not None and rot_axes:
            for r in range(3):
                for c in range(3):
                    if (r + 1) in rot_axes and (c + 1) in rot_axes:
                        cov[(r + 3) * 6 + (c + 3)] = float(rot_cov[r][c])

        return cov.tolist()

    def _build_angular_cov(self):
        """6x6 twist covariance: large values for linear, yaml values for angular."""
        cov = np.zeros(36)
        ang_axes = self.get_axes("angular")
        ang_cov = self.get_covariance("angular")

        for i in range(3):
            cov[i * 6 + i] = _BIG

        if ang_cov is not None and ang_axes:
            for r in range(3):
                for c in range(3):
                    if (r + 1) in ang_axes and (c + 1) in ang_axes:
                        cov[(r + 3) * 6 + (c + 3)] = float(ang_cov[r][c])

        return cov.tolist()

    def _build_accel_cov(self):
        """3x3 linear-acceleration covariance for the Imu message."""
        cov = np.zeros(9)
        accel_axes = self.get_axes("accel")
        accel_cov = self.get_covariance("accel")

        if accel_cov is not None and accel_axes:
            for r in range(3):
                for c in range(3):
                    if (r + 1) in accel_axes and (c + 1) in accel_axes:
                        cov[r * 3 + c] = float(accel_cov[r][c])

        return cov.tolist()

    def publish_sensor_data(self):
        msg = self._latest_msg
        if msg is None:
            return

        stamp = msg.header.stamp

        if self.is_active("rotation"):
            rot_msg = PoseWithCovarianceStamped()
            rot_msg.header.stamp = stamp
            rot_msg.header.frame_id = "base_link"
            rot_msg.pose.pose.orientation = msg.orientation
            rot_msg.pose.covariance = self._build_rotation_cov()
            self.sensor_publishers["rotation"].publish(rot_msg)

        if self.is_active("angular"):
            ang_msg = TwistWithCovarianceStamped()
            ang_msg.header.stamp = stamp
            ang_msg.header.frame_id = "base_link"
            ang_msg.twist.twist.angular = msg.angular_velocity
            ang_msg.twist.covariance = self._build_angular_cov()
            self.sensor_publishers["angular"].publish(ang_msg)

        if self.is_active("accel"):
            accel_msg = Imu()
            accel_msg.header.stamp = stamp
            accel_msg.header.frame_id = "base_link"
            accel_msg.linear_acceleration = msg.linear_acceleration
            accel_msg.linear_acceleration_covariance = self._build_accel_cov()
            accel_msg.orientation_covariance = [-1.0] + [0.0] * 8
            accel_msg.angular_velocity_covariance = [-1.0] + [0.0] * 8
            self.sensor_publishers["accel"].publish(accel_msg)

    def rotate_quaternion(self, w, x, y, z):
        original_data = [x, y, z, w]
        q_original = R.from_quat(original_data)

        r_base = R.from_quat([0.0, 0.7071, 0.0, -0.7071])  # 90 deg around Y
        r_flip = R.from_quat([0.0, 0.0, 1.0, 0.0])
        r_change = r_flip * r_base

        q_new = r_change * q_original * r_change.inv()
        quat_array = q_new.as_quat()
        x, y, z, w = quat_array

        return w, x, y, z


def main(args=None):
    rclpy.init(args=args)
    imu = IMU()
    rclpy.spin(imu)
    imu.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
