import rclpy
from rclpy.node import Node
from msgs.msg import GeneratedPath
from gz.transport13 import Node as GzNode
from gz.custom_msgs.GeneratedPath import GeneratedPath as GzPath
from gz.msgs10.pose_pb2 import Pose as GzPose
from gz.msgs10.twist_pb2 import Twist as GzTwist
from gz.msgs10.vector3d_pb2 import Vector3d
from gz.msgs10.quaternion_pb2 import Quaternion

class PathBridgeNode(Node):
    def __init__(self):
        super().__init__("path_bridge")
        # Subscribe to ROS path topic
        self.subscription = self.create_subscription(
            GeneratedPath,
            "/path",  # Adjust topic name as needed
            self.path_callback,
            10
        )
        
        # Create Gazebo node and publisher
        self.gz_node = GzNode()
        self.gz_publisher = self.gz_node.advertise(GzPath, "/path")

    def path_callback(self, msg: GeneratedPath):
        self.get_logger().info("Received Path message")

        # Create Gazebo Path message
        gz_msg = GzPath()
        
        # Convert each PoseStamped and TwistStamped to Gazebo format
        for pose in msg.poses:
            gz_pose = GzPose()
            
            # Set position
            position = Vector3d()
            position.x = pose.pose.position.x
            position.y = pose.pose.position.y
            position.z = pose.pose.position.z
            gz_pose.position.CopyFrom(position)
            
            # Set orientation
            orientation = Quaternion()
            orientation.x = pose.pose.orientation.x
            orientation.y = pose.pose.orientation.y
            orientation.z = pose.pose.orientation.z
            orientation.w = pose.pose.orientation.w
            gz_pose.orientation.CopyFrom(orientation)
            
            gz_msg.poses.append(gz_pose)

        for twist in msg.twists:
            gz_twist = GzTwist()
            
            # Set linear velocity
            linear = Vector3d()
            linear.x = twist.twist.linear.x
            linear.y = twist.twist.linear.y
            linear.z = twist.twist.linear.z
            gz_twist.linear.CopyFrom(linear)
            
            # Set angular velocity
            angular = Vector3d()
            angular.x = twist.twist.angular.x
            angular.y = twist.twist.angular.y
            angular.z = twist.twist.angular.z
            gz_twist.angular.CopyFrom(angular)
            
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
