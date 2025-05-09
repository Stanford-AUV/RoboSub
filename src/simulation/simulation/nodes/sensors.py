import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import numpy as np

def get_angular_velocity(q1, q2, dt):
    return (2 / dt) * np.array([
        q1[0]*q2[1] - q1[1]*q2[0] - q1[2]*q2[3] + q1[3]*q2[2],
        q1[0]*q2[2] + q1[1]*q2[3] - q1[2]*q2[0] - q1[3]*q2[1],
        q1[0]*q2[3] - q1[1]*q2[2] + q1[2]*q2[1] - q1[3]*q2[0]])

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

        twist = Twist()
        delta_t = (msg.header.stamp.sec - self.last_pose.header.stamp.sec) + (msg.header.stamp.nanosec - self.last_pose.header.stamp.nanosec) / 1e9
        
        # Avoid division by zero
        if delta_t > 0:
            # Linear velocity calculation
            twist.linear.x = (msg.pose.position.x - self.last_pose.pose.position.x) / delta_t
            twist.linear.y = (msg.pose.position.y - self.last_pose.pose.position.y) / delta_t
            twist.linear.z = (msg.pose.position.z - self.last_pose.pose.position.z) / delta_t
            
            # Angular velocity calculation using spatialmath's UnitQuaternion
            current_quat = np.array([
                msg.pose.orientation.w,
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
            ])
            
            last_quat = np.array([
                self.last_pose.pose.orientation.w,
                self.last_pose.pose.orientation.x,
                self.last_pose.pose.orientation.y,
                self.last_pose.pose.orientation.z,
            ])
            
            angular_velocity = get_angular_velocity(last_quat, current_quat, delta_t)
            twist.angular.x = angular_velocity[0]
            twist.angular.y = angular_velocity[1]
            twist.angular.z = angular_velocity[2]
            
            odometry.twist.twist = twist
            self._odometry_pub.publish(odometry)
        
        self.last_pose = msg


def main(args=None):
    rclpy.init(args=args)
    node = Sensors()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
