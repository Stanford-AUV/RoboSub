import rclpy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
import numpy as np
from hardware.nodes.generic_sensor import GenericSensor

from scipy.spatial.transform import Rotation as R

# TODO: calibrate per unit once the sensors are mounted (log stationary gyro
# and average, same procedure as the Xsens bias in imu.py). Indexed by
# imu_index.
GYRO_BIAS = {
    0: np.zeros(3),
    1: np.zeros(3),
}

_BIG = 1e9


class BNO085(GenericSensor):
    """One of the two Bosch BNO085 IMUs, read via the Arduino.

    The arduino node publishes raw sensor-frame data (BNO085 on-chip fusion
    quaternion + gyro + accel) on /arduino/imu_<n>. This node remounts it into
    base_link using R_sensor_to_base from sensors.yaml and republishes under
    /bno085_<n>/... (namespaced: the Xsens owns the bare /rotation, /angular,
    /accel topics).

    Launch twice, once per unit:
        ros2 run hardware bno085_0
        ros2 run hardware bno085_1
    """

    def __init__(self, imu_index):
        super().__init__(f"bno085_{imu_index}", f"bno085_{imu_index}")
        self.imu_index = imu_index

        self.is_initialized = False
        self.rot_init_inv = None

        ns = f"/bno085_{self.imu_index}"
        if self.is_active("rotation"):
            self.sensor_publishers["rotation"] = self.create_publisher(
                PoseWithCovarianceStamped, f"{ns}/rotation", 10
            )
        if self.is_active("angular"):
            self.sensor_publishers["angular"] = self.create_publisher(
                TwistWithCovarianceStamped, f"{ns}/angular", 10
            )
        if self.is_active("accel"):
            self.sensor_publishers["accel"] = self.create_publisher(
                Imu, f"{ns}/accel", 10
            )

        self.imu_subscription = self.create_subscription(
            Imu, f"/arduino/imu_{self.imu_index}", self._imu_callback, 10
        )

        self._latest_msg = None

    def _imu_callback(self, msg: Imu):
        w = np.array(
            [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            dtype=float,
        )
        w -= GYRO_BIAS[self.imu_index]
        w_base = self.R_sensor_to_base @ w
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = (
            w_base.tolist()
        )

        # Rotate linear acceleration into base_link with the same mounting
        # rotation as the gyro (see imu.py for the full rationale).
        # TODO: confirm which BNO085 accel report the firmware forwards.
        # robot_localization's remove_gravitational_acceleration expects
        # gravity INCLUDED (raw accelerometer, reads +9.8 on base +Z at rest);
        # if the firmware sends the linear-acceleration report (gravity
        # already removed), turn that flag off for this input instead.
        a = np.array(
            [
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            ],
            dtype=float,
        )
        a_base = self.R_sensor_to_base @ a
        (
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ) = a_base.tolist()

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
            self.get_logger().warn(
                f"Invalid BNO085_{self.imu_index} quaternion; dropping msg"
            )
            return

        q_raw /= np.linalg.norm(q_raw)

        rot_sensor = R.from_quat(q_raw)

        # Single remount rotation, same convention as imu.py:
        #     world_from_base = world_from_sensor * sensor_from_base
        rot_final = rot_sensor * R.from_matrix(self.R_sensor_to_base).inv()

        # Zero ONLY the initial heading (yaw); keep roll/pitch absolute
        # (gravity-referenced). See imu.py for why zeroing the full attitude
        # couples yaw drift into roll/pitch.
        if not self.is_initialized:
            yaw0 = rot_final.as_euler("ZYX")[0]
            self.rot_init_inv = R.from_euler("z", -yaw0)
            self.is_initialized = True
            self.get_logger().info(
                f"BNO085_{self.imu_index} heading zeroed (roll/pitch kept absolute)"
            )

        rot_zeroed = self.rot_init_inv * rot_final
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
            # World frame, NOT base_link: absolute-orientation pose (see
            # imu.py / depth_sensor.py for the frame_id="odom" rationale).
            rot_msg.header.frame_id = "odom"
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
            # Ship the remounted attitude with the accel so gravity removal
            # uses this message's roll/pitch (see imu.py).
            accel_msg.orientation = msg.orientation
            accel_msg.orientation_covariance = [
                0.001, 0.0, 0.0,
                0.0, 0.001, 0.0,
                0.0, 0.0, 0.05,
            ]
            accel_msg.angular_velocity_covariance = [-1.0] + [0.0] * 8
            self.sensor_publishers["accel"].publish(accel_msg)


def main(imu_index, args=None):
    rclpy.init(args=args)
    node = BNO085(imu_index)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


def main_0(args=None):
    main(0, args=args)


def main_1(args=None):
    main(1, args=args)


if __name__ == "__main__":
    main(0)
