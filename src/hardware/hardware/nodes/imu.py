import rclpy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import QuaternionStamped
import numpy as np
from hardware.nodes.generic_sensor import GenericSensor

from scipy.spatial.transform import Rotation as R


class IMU(GenericSensor):
    """Republish the Xsens on-board filter quaternion as a plain Imu message.

    We trust the Xsens EKF's fused/filtered orientation and forward ONLY that
    (remounted into base_link, initial heading zeroed) for the downstream EKF to
    fuse. Angular velocity and linear acceleration are marked unused (cov[0]=-1).
    """

    def __init__(self):
        super().__init__("imu", "imu_0")

        self.is_initialized = False
        self.rot_init_inv = None

        # Throttle the republish to the downstream EKF. The Xsens streams at
        # output_data_rate (400Hz); forwarding that 1:1 floods robot_localization's
        # single-threaded executor, which drains the IMU callback backlog before it
        # can run its 50Hz filter step -> multi-second filter freezes (observed
        # 0.5-6.8s stalls in bag run_20260712_010321). The EKF runs at 50Hz, so
        # ~100Hz orientation is ample margin. Gate on the sensor timestamp so the
        # rate tracks device time, not processing jitter. Tunable via param.
        self.declare_parameter("orientation_pub_rate_hz", 100.0)
        rate_hz = self.get_parameter("orientation_pub_rate_hz").value
        self._min_pub_period = (1.0 / rate_hz) if rate_hz > 0.0 else 0.0
        self._last_pub_t = None

        # NOT "/imu/data": the Xsens driver already owns that topic (its own raw
        # sensor_msgs/Imu). We publish the remounted, heading-zeroed orientation
        # on a distinct topic for the EKF.
        self.imu_publisher = self.create_publisher(Imu, "/imu/orientation", 10)

        # Driver publishes geometry_msgs/QuaternionStamped here (no covariance).
        # The name is ABSOLUTE in the driver (orientationpublisher.h:47), so no
        # namespace is applied -> it is "/filter/quaternion", not "/xsens/...".
        self.imu_subscription = self.create_subscription(
            QuaternionStamped, "/filter/quaternion", self._imu_callback, 10
        )

    def _imu_callback(self, msg: QuaternionStamped):
        q_raw = np.array(
            [
                msg.quaternion.x,
                msg.quaternion.y,
                msg.quaternion.z,
                msg.quaternion.w,
            ],
            dtype=float,
        )
        if (not np.all(np.isfinite(q_raw))) or (np.linalg.norm(q_raw) < 1e-6):
            self.get_logger().warn("Invalid IMU quaternion; dropping msg")
            return

        # Rate-limit forwarding to the EKF (see __init__). The first valid msg
        # always passes so heading-zero init still runs; afterwards we skip any
        # msg arriving less than _min_pub_period after the last forwarded one.
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if (
            self._last_pub_t is not None
            and (stamp - self._last_pub_t) < self._min_pub_period
        ):
            return
        self._last_pub_t = stamp

        q_raw /= np.linalg.norm(q_raw)
        rot_sensor = R.from_quat(q_raw)

        # Remount the sensor orientation into base_link (FLU):
        #     world_from_base = world_from_sensor * sensor_from_base
        # with sensor_from_base = R_sensor_to_base^-1. Puts gravity on base +Z,
        # so roll/pitch are the gravity-referenced tilt axes and yaw is heading.
        rot_final = rot_sensor * R.from_matrix(self.R_sensor_to_base).inv()

        # Zero ONLY the initial heading (yaw); keep roll/pitch absolute
        # (gravity-referenced) so turning the vehicle never couples into tilt.
        if not self.is_initialized:
            yaw0 = rot_final.as_euler("ZYX")[0]
            self.rot_init_inv = R.from_euler("z", -yaw0)
            self.is_initialized = True
            self.get_logger().info("IMU heading zeroed (roll/pitch kept absolute)")

        rot_zeroed = self.rot_init_inv * rot_final
        qx, qy, qz, qw = rot_zeroed.as_quat()

        out = Imu()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = "base_link"
        out.orientation.x = float(qx)
        out.orientation.y = float(qy)
        out.orientation.z = float(qz)
        out.orientation.w = float(qw)
        out.orientation_covariance = self._orientation_cov()
        # Mark angular velocity + linear acceleration as unused for the EKF.
        out.angular_velocity_covariance = [-1.0] + [0.0] * 8
        out.linear_acceleration_covariance = [-1.0] + [0.0] * 8

        self.imu_publisher.publish(out)

    def _orientation_cov(self):
        """3x3 orientation covariance (row-major 9) from sensors.yaml."""
        rot_cov = self.get_covariance("rotation")
        if rot_cov is None:
            return [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.05]
        cov = np.zeros(9)
        for r in range(3):
            for c in range(3):
                cov[r * 3 + c] = float(rot_cov[r][c])
        return cov.tolist()


def main(args=None):
    rclpy.init(args=args)
    imu = IMU()
    rclpy.spin(imu)
    imu.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
