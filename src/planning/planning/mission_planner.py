import rclpy
from rclpy.node import Node
from rclpy import Parameter

import math
from typing import Optional

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from msgs.msg import GeneratedPath  # Custom message import
from vision_msgs.msg import BoundingBox3D


class Mission(Node):
    def __init__(self):
        super().__init__('mission')

        # Declare parameters
        self.declare_parameter('waypoints_topic', 'waypoints')
        self.declare_parameter("history_depth", Parameter.Type.INTEGER)

        # Get parameters
        waypoints_topic = self.get_parameter('waypoints_topic').value
        history_depth = (
            self.get_parameter("history_depth").get_parameter_value().integer_value
        )

        # Set up publisher for waypoints topic
        self.waypoints_publisher = self.create_publisher(Path, waypoints_topic, history_depth)

        # Set up subscriber for generated path
        self.waypoints_subscriber = self.create_subscription(
            GeneratedPath, "generated_path", self.generated_path_callback, history_depth
        )

        # Set up subscribers to receive PoseStamped and Twist for position, orientation, and velocity
        self.pose_subscriber = self.create_subscription(PoseStamped, "current_pose", self.pose_callback, 10)
        self.twist_subscriber = self.create_subscription(Twist, "current_velocity", self.twist_callback, 10)

        # Variables to store the latest pose and twist messages
        self.current_pose = PoseStamped()
        self.current_velocity = Twist()

    def publish_waypoints(self, path: Path):
        """Publish the waypoints on the waypoints topic."""
        self.waypoints_publisher.publish(path)
        self.get_logger().info("Published waypoints path.")

    def generated_path_callback(self, msg: GeneratedPath):
        """Callback for receiving GeneratedPath messages."""
        self.get_logger().info("Received generated path.")

    def run(self):
        raise NotImplementedError("Subclass must implement this method")

    def followPath(self, path: Path) -> None:
        self.publish_waypoints(path)

    def rotate(self, twist: Twist) -> None:
        raise NotImplementedError("Subclass must implement this method")

    ### Method to rotate to then around axis until object found ###
    ### Method to rotate around Fibonacci spehre spiral until object found (50 degrees FOV) ###

    def getObjectPosition(self, tag: str) -> Optional[BoundingBox3D]:
        raise NotImplementedError("Subclass must implement this method")

    def getDirection(self, pose: PoseStamped) -> Vector3:
        raise NotImplementedError("Subclass must implement this method")

    def getPositionAndOrientation(self) -> PoseStamped:
        """Returns the latest PoseStamped message for position and orientation."""
        return self.current_pose

    def getAngularAndLinearVelocity(self) -> Twist:
        """Returns the latest Twist message for angular and linear velocity."""
        return self.current_velocity

    def getDistanceFromPose(self, target_pose: PoseStamped) -> float:
        """Calculate the Euclidean distance from the current pose to the target pose."""
        dx = target_pose.pose.position.x - self.current_pose.pose.position.x
        dy = target_pose.pose.position.y - self.current_pose.pose.position.y
        dz = target_pose.pose.position.z - self.current_pose.pose.position.z
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        self.get_logger().info(f"Calculated distance: {distance}")
        return distance

    def pose_callback(self, msg: PoseStamped):
        """Callback to update the current pose."""
        self.current_pose = msg
        self.get_logger().info("Updated current pose.")

    def twist_callback(self, msg: Twist):
        """Callback to update the current velocity."""
        self.current_velocity = msg
        self.get_logger().info("Updated current velocity.")

# Running the node (useful if standalone)
def main(args=None):
    rclpy.init(args=args)
    mission = Mission()
    rclpy.spin(mission)
    mission.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
