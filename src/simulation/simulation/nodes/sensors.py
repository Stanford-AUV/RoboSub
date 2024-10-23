import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from ros_gz_interfaces.msg import Altimeter
from geometry_msgs.msg import PoseStamped


class Sensors(Node):
    """This node converts sensor messages sent from Gazebo into a unified format for use for state estimation and control."""

    def __init__(self):
        super().__init__("sensors")

        self._imu_sub = self.create_subscription(Imu, "gz/imu", self.imu_callback, 10)
        self._imu_pub = self.create_publisher(Imu, "imu", 10)

        self._altimeter_sub = self.create_subscription(
            Altimeter, "gz/depth", self.altimeter_callback, 10
        )
        self._altimeter_pub = self.create_publisher(Altimeter, "depth", 10)

        self._pose_sub = self.create_subscription(
            PoseStamped, "gz/pose", self.pose_callback, 10
        )
        self._pose_pub = self.create_publisher(PoseStamped, "pose", 10)

    def imu_callback(self, msg: Imu):
        self.get_logger().info("Received IMU message")
        # Since underwater, remove g from the acceleration z-axis
        msg.linear_acceleration.z -= 9.8
        self._imu_pub.publish(msg)

    def altimeter_callback(self, msg: Altimeter):
        self.get_logger().info("Received altimeter message")
        self._altimeter_pub.publish(msg)

    def pose_callback(self, msg: PoseStamped):
        self.get_logger().info(f"Received pose message: {msg.pose}")
        self._pose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Sensors()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
