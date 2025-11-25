import os
import sys

# Add the correct lib/python path for custom msgs
sys.path.append(
    "/workspaces/RoboSub/build/custom_gz_plugins/custom_gz_plugins-msgs_genmsg/python"
)

import rclpy
from rclpy.node import Node
from msgs.msg import GeneratedPath as ROSGeneratedPath
from gz.msgs10.vector3d_pb2 import Vector3d
from gz.msgs10.quaternion_pb2 import Quaternion
from gz.msgs10.twist_pb2 import Twist
from gz.msgs10.pose_pb2 import Pose
from gz.transport13 import Node as GzNode
from gz.custom_msgs.GeneratedPath_pb2 import GeneratedPath as GZGeneratedPath


class PathBridgeNode(Node):
    def __init__(self):
        print(sys.path)
        super().__init__("path_bridge")
        # Subscribe to ROS path topic
        self.subscription = self.create_subscription(
            ROSGeneratedPath,
            "/generated_path",  # Adjust topic name as needed
            self.path_callback,
            10,
        )

        # Create Gazebo node and publisher
        self.gz_node = GzNode()
        self.gz_publisher = self.gz_node.advertise(
            "/generated_path",
            GZGeneratedPath,
        )

    def path_callback(self, msg: ROSGeneratedPath):
        self.get_logger().info("Received Path message: Here it is:")
        self.get_logger().info(f"{msg}")

        # Create Gazebo Path message
        gz_msg = GZGeneratedPath()

        # Convert each PoseStamped and TwistStamped to Gazebo format
        for pose in msg.poses:
            gz_pose = Pose()
            # Set position
            gz_pose.position.x = pose.pose.position.x
            gz_pose.position.y = pose.pose.position.y
            gz_pose.position.z = pose.pose.position.z
            # Set orientation
            gz_pose.orientation.x = pose.pose.orientation.x
            gz_pose.orientation.y = pose.pose.orientation.y
            gz_pose.orientation.z = pose.pose.orientation.z
            gz_pose.orientation.w = pose.pose.orientation.w
            gz_msg.poses.append(gz_pose)

        for twist in msg.twists:
            gz_twist = Twist()
            # Set linear velocity
            gz_twist.linear.x = twist.linear.x
            gz_twist.linear.y = twist.linear.y
            gz_twist.linear.z = twist.linear.z
            # Set angular velocity
            gz_twist.angular.x = twist.angular.x
            gz_twist.angular.y = twist.angular.y
            gz_twist.angular.z = twist.angular.z
            gz_msg.twists.append(gz_twist)

        # Publish the Gazebo message
        self.gz_publisher.publish(gz_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
