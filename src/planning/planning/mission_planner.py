import rclpy
from rclpy.node import Node
from rclpy import Parameter

import math
from typing import Optional

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from msgs.msg import GeneratedPath  # Custom message import
from vision_msgs.msg import BoundingBox3D

from utils.utils import *

from visualization_msgs.msg import Marker
import matplotlib.pyplot as plt
import numpy as np

class Mission(Node):
    def __init__(self):
        super().__init__("mission")

        # Declare parameters
        self.declare_parameter("waypoints_topic", "waypoints")
        self.declare_parameter("history_depth", 10)

        # Get parameters
        waypoints_topic = self.get_parameter("waypoints_topic").value
        history_depth = (
            self.get_parameter("history_depth").get_parameter_value().integer_value
        )

        # Set up publisher for waypoints topic
        self.waypoints_publisher = self.create_publisher(
            Path, waypoints_topic, history_depth
        )

        # Set up subscriber for generated path
        self.waypoints_subscriber = self.create_subscription(
            GeneratedPath, "generated_path", self.generated_path_callback, history_depth
        )

        # Set up subscribers to receive PoseStamped and Twist for position, orientation, and velocity
        self.pose_subscriber = self.create_subscription(
            PoseStamped, "current_pose", self.pose_callback, 10
        )
        self.twist_subscriber = self.create_subscription(
            Twist, "current_velocity", self.twist_callback, 10
        )

        # Variables to store the latest pose and twist messages
        self.current_pose = PoseStamped()
        self.current_velocity = Twist()

    def generate_path(self, input_path: Path):
        """
        Generate a new path based on the input path, publish it, and process the response.

        Args:
            input_path (Path): The input path to process and generate the new path.
        """
        # Log the input path for debugging
        self.get_logger().info(
            f"Received input path with {len(input_path.poses)} waypoints."
        )

        # Publish the input path
        self.publish_waypoints(input_path)

        # Wait for the generated path to be received (optional, could use a flag or event)
        self.get_logger().info("Waiting for generated path...")
        timeout = 100  # Timeout in seconds
        start_time = self.get_clock().now().nanoseconds
        while not hasattr(self, "last_generated_path"):
            current_time = self.get_clock().now().nanoseconds
            if (current_time - start_time) / 1e9 > timeout:
                self.get_logger().error("Timeout while waiting for generated path.")
                return None
            rclpy.spin_once(self, timeout_sec=0.1)

        # Process the received generated path
        generated_path = self.last_generated_path
        self.get_logger().info(
            f"Generated path received with {len(generated_path.poses)} waypoints."
        )

        return generated_path

    def follow_path(self, generated_path: GeneratedPath):
        raise NotImplementedError("TODO Later")

    def look_around(self):
        """
        Make the robot look around one revolution along its z axis.
        """
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        directions = [(0.0, 1.0, 0.0), (0.0, 1.0, 1.0), (0.0, 0.0, 1.0)]
        for direction in directions:
            # Convert spherical coordinates to quaternion for orientation
            quaternion = direction_to_quaternion(direction)
            print(quaternion)

            # Create a PoseStamped for the orientation
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]

            # Append to the path
            path_msg.poses.append(pose)

        self.generate_path(path_msg)
        self.get_logger().info("Completed looking in all directions.")

    def look_in_all_directions(self, overlap_percentage: float = 0.2):
        """
        Make the robot look in all directions using a Fibonacci sphere path.

        Args:
            overlap_percentage (float): The percentage overlap between consecutive views.
        """
        self.get_logger().info(
            f"Starting to look in all directions with {overlap_percentage*100:.1f}% overlap."
        )

        # Camera FOV in radians
        fov_radians = math.radians(50)
        half_fov = fov_radians / 2

        # Calculate the number of points needed for full coverage with overlap
        required_angle = fov_radians * (1 - overlap_percentage)
        num_points = math.ceil(4 * math.pi / (required_angle**2))

        self.get_logger().info(f"Using {num_points} points to cover the sphere.")

        # Generate Fibonacci sphere points
        directions = generate_fibonacci_sphere_points(26)

        # Create a Path message for the directions
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for direction in directions:
            # Convert spherical coordinates to quaternion for orientation
            quaternion = direction_to_quaternion(direction)

            # Create a PoseStamped for the orientation
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]

            # Append to the path
            path_msg.poses.append(pose)

        # Use the `generate_path` method to publish and follow the generated path
        self.generate_path(path_msg)

        ##########################
        # TODO: FOLLOW THE PATH #
        ##########################

        self.get_logger().info("Completed looking in all directions.")

    def publish_waypoints(self, path: Path):
        """Publish the waypoints on the waypoints topic."""
        self.waypoints_publisher.publish(path)
        self.get_logger().info("Published waypoints path.")

    def generated_path_callback(self, msg: GeneratedPath):
        """Callback for receiving GeneratedPath messages."""
        
        self.get_logger().info("Received generated path.")

        # Extract the poses from the message
        poses = msg.poses
        
        # Extract the x, y, z coordinates and orientation from the poses
        x_vals = []
        y_vals = []
        z_vals = []
        roll_vals = []
        pitch_vals = []
        yaw_vals = []

        for pose in poses:
            # Position data
            x_vals.append(pose.pose.position.x)
            y_vals.append(pose.pose.position.y)
            z_vals.append(pose.pose.position.z)

            # Orientation data (quaternion to Euler angles)
            orientation = pose.pose.orientation
            q_x, q_y, q_z, q_w = orientation.x, orientation.y, orientation.z, orientation.w

            # Convert quaternion to roll, pitch, yaw (in radians)
            # Roll (x-axis rotation)
            roll = np.arctan2(2 * (q_w * q_x + q_y * q_z), 1 - 2 * (q_x**2 + q_y**2))

            # Pitch (y-axis rotation)
            pitch = np.arcsin(2 * (q_w * q_y - q_z * q_x))

            # Yaw (z-axis rotation)
            yaw = np.arctan2(2 * (q_w * q_z + q_x * q_y), 1 - 2 * (q_y**2 + q_z**2))

            # Store the Euler angles (tilt values)
            roll_vals.append(roll)
            pitch_vals.append(pitch)
            yaw_vals.append(yaw)

        # Convert lists to numpy arrays for plotting
        roll_vals = np.array(roll_vals)
        pitch_vals = np.array(pitch_vals)
        yaw_vals = np.array(yaw_vals)

        # Create the 3D plot for orientation
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot roll, pitch, and yaw in a 3D space
        ax.plot(roll_vals, pitch_vals, yaw_vals, marker='o', linestyle='-', color='b')
        
        ax.set_xlabel('Roll (rad)')
        ax.set_ylabel('Pitch (rad)')
        ax.set_zlabel('Yaw (rad)')
        ax.set_title('Orientation (Roll, Pitch, Yaw) Over Time')

        # Save the 3D orientation plot
        fig.savefig("orientation_3d_plot.png", dpi=300)  # Save as PNG with high resolution

        # Show the plot
        plt.show()


        
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
