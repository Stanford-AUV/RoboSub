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
        
        self._pose_sub = self.create_subscription(
            PoseStamped, "gz/pose", self.pose_callback, 10
        )
        self._odometry_pub = self.create_publisher(Odometry, "/odometry/filtered", 10)

        self.last_pose = PoseStamped()

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
