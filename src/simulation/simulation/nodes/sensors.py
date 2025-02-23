import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from ros_gz_interfaces.msg import Altimeter
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry


class Sensors(Node):
    """This node converts sensor messages sent from Gazebo into a unified format for use for state estimation and control."""

    def __init__(self):
        super().__init__("sensors")

        self._imu_sub = self.create_subscription(Imu, "gz, self.imu_callback, 10)
        self._imu_pub = self.create_publisher(Imu, "imu", 10)

        self._altimeter_sub = self.create_subscription(
            Altimeter, "gz/depth", self.altimeter_callback, 10
        )
        self._altimeter_pub = self.create_publisher(Altimeter, "depth", 10)

        self._pose_sub = self.create_subscription(
            PoseStamped, "gz/pose", self.pose_callback, 10
        )
        self._odometry_pub = self.create_publisher(Odometry, "odometry", 10)

        self.last_pose = PoseStamped()

    def imu_callback(self, msg: Imu):
        self.get_logger().info("Received IMU message")
        # Since underwater, remove g from the acceleration z-axis
        msg.linear_acceleration.z -= 9.8
        self._imu_pub.publish(msg)

    def altimeter_callback(self, msg: Altimeter):
        self.get_logger().info("Received altimeter message")
        self._altimeter_pub.publish(msg)

    def pose_callback(self, msg: PoseStamped):
        odometry = Odometry()
        odometry.header.stamp = msg.header.stamp
        odometry.pose.pose = msg.pose

        twist = TwistStamped()
        twist.header.stamp = msg.header.stamp
        twist.twist.linear.x = (
            (msg.pose.position.x - self.last_pose.pose.position.x)
            * 1e9
            / (msg.header.stamp.nanosec - self.last_pose.header.stamp.nanosec)
        )
        twist.twist.linear.y = (
            (msg.pose.position.y - self.last_pose.pose.position.y)
            * 1e9
            / (msg.header.stamp.nanosec - self.last_pose.header.stamp.nanosec)
        )
        twist.twist.linear.z = (
            (msg.pose.position.z - self.last_pose.pose.position.z)
            * 1e9
            / (msg.header.stamp.nanosec - self.last_pose.header.stamp.nanosec)
        )
        twist.twist.angular.x = (
            (self.last_pose.pose.orientation.x - msg.pose.orientation.x)
            / (msg.header.stamp.nanosec - self.last_pose.header.stamp.nanosec)
            * 1e9
        )
        twist.twist.angular.y = (
            (self.last_pose.pose.orientation.y - msg.pose.orientation.y)
            * 1e9
            / (msg.header.stamp.nanosec - self.last_pose.header.stamp.nanosec)
        )
        twist.twist.angular.z = (
            (self.last_pose.pose.orientation.z - msg.pose.orientation.z)
            * 1e9
            / (msg.header.stamp.nanosec - self.last_pose.header.stamp.nanosec)
        )

        self.get_logger().info(f"Received odometry message: {odometry}")
        self.last_pose = msg
        self._odometry_pub.publish(odometry)


def main(args=None):
    rclpy.init(args=args)
    node = Sensors()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
