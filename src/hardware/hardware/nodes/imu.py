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
        self.rot_init_inv = None

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

        # Rotate linear acceleration into base_link with the same mounting
        # rotation as the gyro. Without this the /accel message claims
        # frame_id="base_link" while carrying sensor-frame data, so the EKF
        # would fuse surge/sway/heave accelerations on the wrong axes.
        # (Xsens /imu/data accel includes gravity: reads +9.8 on base +Z at
        # rest after this rotation, which is what robot_localization's
        # remove_gravitational_acceleration expects.)
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
            self.get_logger().warn("Invalid IMU quaternion; dropping msg")
            return

        q_raw /= np.linalg.norm(q_raw)

        rot_sensor = R.from_quat(q_raw)

        # Remount the sensor orientation into the base_link (FLU) frame with a
        # single rotation:
        #     world_from_base = world_from_sensor * sensor_from_base
        # where sensor_from_base = R_sensor_to_base^-1. This puts gravity on base
        # +Z, so roll/pitch are the (gravity-referenced) tilt axes and yaw is
        # heading -- verified against the accelerometer, which reads ~9.8 on Z
        # in the base frame (R_sensor_to_base @ [0.1, 9.8, 0.8] = [0.8, 0.1, 9.8]).
        #
        # This replaces the old chain (rotate_quaternion: 90 deg about Y + a flip,
        # then 180 deg about Z), which left the frame mis-rotated ~90 deg and
        # landed gravity/heading on the PITCH axis. That is why the VRU's
        # unreferenced heading drift showed up as a slow pitch ramp instead of
        # yaw, and why a tilted vehicle coupled that drift into pitch/roll.
        rot_final = rot_sensor * R.from_matrix(self.R_sensor_to_base).inv()

        # Zero ONLY the initial heading (yaw); keep roll/pitch ABSOLUTE
        # (gravity-referenced). Zeroing the full orientation makes the reference
        # frame the startup ATTITUDE -- if the sub isn't perfectly level at init,
        # a real yaw (rotation about true vertical) then bleeds into roll/pitch.
        # Removing only heading as a world-frame +Z rotation cannot tilt the
        # reference, so turning the vehicle never couples into roll/pitch.
        if not self.is_initialized:
            yaw0 = rot_final.as_euler("ZYX")[0]  # initial heading about world +Z
            self.rot_init_inv = R.from_euler("z", -yaw0)
            self.is_initialized = True
            self.get_logger().info("IMU heading zeroed (roll/pitch kept absolute)")

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
            # World frame, NOT base_link: this is an absolute-orientation pose.
            # robot_localization transforms a pose into world_frame (odom); with
            # frame_id="base_link" it needs an odom<-base_link tf that doesn't
            # exist until the EKF initializes -> deadlock, every measurement
            # "Could not transform measurement into odom. Ignoring..." -> no output.
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
            # Include the (remounted, yaw-zeroed) orientation so
            # robot_localization removes gravity with THIS message's attitude
            # instead of falling back to (possibly lagged) filter state.
            # Gravity removal only depends on roll/pitch, so the yaw-zeroing
            # is irrelevant here. imu0_config keeps orientation fusion off in
            # ekf.yaml, so this is not double-fused with /rotation.
            accel_msg.orientation = msg.orientation
            accel_msg.orientation_covariance = [
                0.001, 0.0, 0.0,
                0.0, 0.001, 0.0,
                0.0, 0.0, 0.05,
            ]
            accel_msg.angular_velocity_covariance = [-1.0] + [0.0] * 8
            self.sensor_publishers["accel"].publish(accel_msg)


def main(args=None):
    rclpy.init(args=args)
    imu = IMU()
    rclpy.spin(imu)
    imu.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
