import rclpy
from sensor_msgs.msg import Imu
import numpy as np
from hardware.nodes.generic_sensor import GenericSensor

GYRO_BIAS = np.array([
    -0.006099891562248375,
    -0.0034746338098969654,
    0.00273246756079096,
], dtype=float)


class IMU(GenericSensor):
    def __init__(self):
        super().__init__("imu", "imu_0")

        self._imu_pub = self.create_publisher(Imu, "/imu", 10)

        self.imu_subscription = self.create_subscription(
            Imu, "/imu/data", self.imu_listener_callback, 10
        )

    def imu_listener_callback(self, msg: Imu):

        w = np.array([out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z], dtype=float)
        w -= GYRO_BIAS
        out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = w.tolist()

        if not out.header.frame_id:
            out.header.frame_id = "imu_frame"

        q = np.array([out.orientation.x, out.orientation.y,
                       out.orientation.z, out.orientation.w], dtype=float)
        if (not np.all(np.isfinite(q))) or (np.linalg.norm(q) < 1e-6):
            self.get_logger().warn("Invalid IMU quaternion; dropping msg")
            return

        q /= np.linalg.norm(q)
        out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w = q.tolist()

        self._imu_pub.publish(out)

    def publish_sensor_data(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    imu = IMU()
    rclpy.spin(imu)
    imu.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
