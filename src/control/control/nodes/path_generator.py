import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist
from msgs.msg import GeneratedPath  # Custom message import
from rclpy import Parameter

import numpy as np
from scipy.spatial.transform import Rotation

from control.utils.create_path import create_path


class PathGenerator(Node):
    def __init__(self):
        super().__init__("path_generator")

        self.declare_parameter("waypoints_topic", "waypoints")
        self.declare_parameter("history_depth", Parameter.Type.INTEGER)

        history_depth = (
            self.get_parameter("history_depth").get_parameter_value().integer_value
        )

        self.waypoints_subscriber = self.create_subscription(
            Path, "waypoints", self.waypoints_callback, history_depth
        )  # Create test waypoints node

        self.generated_path_publisher = self.create_publisher(
            GeneratedPath, "generated_path", history_depth
        )

        self.get_logger().info("PathGenerator node has been started.")

    def waypoints_callback(self, msg: Path):
        self.get_logger().info("Received waypoints, generating path...")

        x_positions = []
        y_positions = []
        z_positions = []
        roll_angles = []
        pitch_angles = []
        yaw_angles = []

        # Loop through each pose in the Path message
        for pose_stamped in msg.poses:
            # Extract positions
            x_positions.append(pose_stamped.pose.position.x)
            y_positions.append(pose_stamped.pose.position.y)
            z_positions.append(pose_stamped.pose.position.z)

            # Extract orientations in quaternion form
            orientation_q = pose_stamped.pose.orientation
            quaternion = [
                orientation_q.x,
                orientation_q.y,
                orientation_q.z,
                orientation_q.w,
            ]

            # Convert quaternion to Euler angles
            euler = Rotation.from_quat(quaternion).as_euler("xyz", degrees=True)
            roll_angles.append(euler[0])  # x Euler angle (roll)
            pitch_angles.append(euler[1])  # y Euler angle (pitch)
            yaw_angles.append(euler[2])  # z Euler angle (yaw)

        # Convert lists to numpy arrays for easier manipulation
        x_positions = np.array(x_positions)
        y_positions = np.array(y_positions)
        z_positions = np.array(z_positions)
        roll_angles = np.array(roll_angles)
        pitch_angles = np.array(pitch_angles)
        yaw_angles = np.array(yaw_angles)

        (
            positions,
            velocities,
            accelerations,
            orientations,
            angular_velocities,
            angular_accelerations,
        ) = create_path(
            x_positions, y_positions, z_positions, roll_angles, pitch_angles, yaw_angles
        )

        self.publish_generated_path(
            positions,
            velocities,
            accelerations,
            orientations,
            angular_velocities,
            angular_accelerations,
        )

    def publish_generated_path(
        self,
        positions,
        velocities,
        accelerations,
        orientations,
        angular_velocities,
        angular_accelerations,
    ):
        # NOTE: Positional shapes are n by 3, rotational shapes are 3 by n
        self.get_logger().info("Generated path, sending back...")

        generated_path = GeneratedPath()  # Initialize the custom message
        generated_path.header.stamp = self.get_clock().now().to_msg()
        generated_path.header.frame_id = "map"  # Set the frame ID appropriately

        # Loop through each position and orientation to populate PoseStamped and Twist messages
        for i in range(len(positions[0])):
            # Create and fill PoseStamped for position and orientation
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = generated_path.header.stamp
            pose_stamped.header.frame_id = generated_path.header.frame_id

            pose_stamped.pose.position.x = positions[0][i]
            pose_stamped.pose.position.y = positions[1][i]
            pose_stamped.pose.position.z = positions[2][i]

            # Convert orientation (roll, pitch, yaw) back to quaternion for the Pose message
            orientation_quat = Rotation.from_euler(
                "xyz", orientations[i], degrees=True
            ).as_quat()
            pose_stamped.pose.orientation.x = orientation_quat[0]
            pose_stamped.pose.orientation.y = orientation_quat[1]
            pose_stamped.pose.orientation.z = orientation_quat[2]
            pose_stamped.pose.orientation.w = orientation_quat[3]

            # Append pose to generated_path.poses
            generated_path.poses.append(pose_stamped)

            # Create and fill Twist for velocity and angular velocity
            twist = Twist()
            twist.linear.x = velocities[0][i]
            twist.linear.y = velocities[1][i]
            twist.linear.z = velocities[2][i]
            twist.angular.x = angular_velocities[i][0]
            twist.angular.y = angular_velocities[i][1]
            twist.angular.z = angular_velocities[i][2]

            # Append twist to generated_path.twists
            generated_path.twists.append(twist)

        # Publish the generated path
        self.generated_path_publisher.publish(generated_path)
        self.get_logger().info("Published generated path with poses and twists.")


def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
