import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from msgs.msg import GeneratedPath  # Custom message import

import os
import yaml
import numpy as np
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory

from planning.utils.create_path import create_path


class PathGenerator(Node):
    def __init__(self):
        super().__init__("path_generator")

        # Waypoints are loaded IN-PROCESS from YAML at startup - there is no
        # /waypoints topic and no separate path_loader node. This deliberately
        # removes the DDS discovery race that used to drop the (one-shot,
        # latched) waypoints message when the loader published before this
        # node's subscription had matched, leaving the generator idle forever.
        # Override the file via the 'waypoints_path' ROS parameter if needed.
        # The YAML is installed into the package share dir (see setup.py), so
        # resolve it via ament - works under any install layout.
        default_yaml = os.path.join(
            get_package_share_directory("planning"), "prequal.yaml"
        )
        self.declare_parameter("waypoints_path", default_yaml)
        self.waypoints_path = (
            self.get_parameter("waypoints_path").get_parameter_value().string_value
            or default_yaml
        )

        self.publish_desired = self.create_publisher(Odometry, "/desired/pose", 10)

        self.generated_path = None
        self.path_start_time = None

        self.create_timer(1.0 / 60.0, self.publish_pose)

        self.get_logger().info("PathGenerator node has been started.")
        self.load_and_generate()

    def load_and_generate(self):
        """Read the waypoint YAML and build the trajectory. Runs once at startup."""
        self.get_logger().info(f"Loading waypoints from {self.waypoints_path}")

        try:
            with open(self.waypoints_path, "r") as f:
                data = yaml.safe_load(f)
        except Exception as exc:
            self.get_logger().error(
                f"Failed to read waypoints file '{self.waypoints_path}': {exc}"
            )
            return

        segments = [data[key] for key in data]

        x_positions = []
        y_positions = []
        z_positions = []
        roll_angles = []
        pitch_angles = []
        yaw_angles = []

        n_waypoints = 0
        for segment in segments:
            for waypoint in segment["waypoints"]:
                x_positions.append(waypoint["position"]["x"])
                y_positions.append(waypoint["position"]["y"])
                z_positions.append(waypoint["position"]["z"])
                roll_angles.append(waypoint["orientation"]["roll"])
                pitch_angles.append(waypoint["orientation"]["pitch"])
                yaw_angles.append(waypoint["orientation"]["yaw"])
                n_waypoints += 1

        if n_waypoints == 0:
            self.get_logger().error(
                f"No waypoints found in '{self.waypoints_path}'; nothing to generate."
            )
            return

        self.get_logger().info(
            f"Loaded {n_waypoints} waypoints from {len(segments)} segment(s), generating path..."
        )

        x_positions = np.array(x_positions)
        y_positions = np.array(y_positions)
        z_positions = np.array(z_positions)
        roll_angles = np.array(roll_angles)
        pitch_angles = np.array(pitch_angles)
        yaw_angles = np.array(yaw_angles)

        try:
            (
                positions,
                velocities,
                accelerations,
                orientations,
                angular_velocities,
                angular_accelerations,
                duration,
            ) = create_path(
                x_positions,
                y_positions,
                z_positions,
                roll_angles,
                pitch_angles,
                yaw_angles,
            )
        except Exception as exc:
            self.get_logger().error(f"Failed to generate path from waypoints: {exc}")
            return

        self.make_generated_path(
            positions,
            velocities,
            accelerations,
            orientations,
            angular_velocities,
            angular_accelerations,
            duration,
        )

    def make_generated_path(
        self,
        positions,
        velocities,
        accelerations,
        orientations,
        angular_velocities,
        angular_accelerations,
        duration,
    ):

        self.get_logger().info("Generated path, sending back...")

        generated_path = GeneratedPath()
        generated_path.header.stamp = self.get_clock().now().to_msg()
        generated_path.header.frame_id = "map"
        generated_path.duration = duration

        for i in range(len(positions[0])):
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = generated_path.header.stamp
            pose_stamped.header.frame_id = generated_path.header.frame_id

            pose_stamped.pose.position.x = positions[0][i]
            pose_stamped.pose.position.y = positions[1][i]
            pose_stamped.pose.position.z = positions[2][i]

            orientation_quat = Rotation.from_euler(
                "xyz", orientations[i], degrees=True
            ).as_quat()
            pose_stamped.pose.orientation.x = orientation_quat[0]
            pose_stamped.pose.orientation.y = orientation_quat[1]
            pose_stamped.pose.orientation.z = orientation_quat[2]
            pose_stamped.pose.orientation.w = orientation_quat[3]

            generated_path.poses.append(pose_stamped)

            twist = Twist()
            twist.linear.x = velocities[0][i]
            twist.linear.y = velocities[1][i]
            twist.linear.z = velocities[2][i]
            twist.angular.x = angular_velocities[i][0]
            twist.angular.y = angular_velocities[i][1]
            twist.angular.z = angular_velocities[i][2]

            generated_path.twists.append(twist)

        self.generated_path = generated_path
        self.path_start_time = self.get_clock().now()
        self.get_logger().info("Generated path with poses and twists.")

    def publish_pose(self):
        if self.generated_path is None or self.path_start_time is None:
            # No trajectory yet - make the idle state loud instead of silent so a
            # failed load is obvious in the terminal (throttled to avoid spam).
            self.get_logger().warn(
                "No generated path yet; not publishing /desired/pose",
                throttle_duration_sec=5.0,
            )
            return

        elapsed = (self.get_clock().now() - self.path_start_time).nanoseconds / 1e9
        duration = self.generated_path.duration
        n = len(self.generated_path.poses)

        if duration <= 0.0 or n == 0:
            return

        t = min(max(elapsed, 0.0), duration)
        index = int(t / duration * (n - 1))
        index = min(index, n - 1)

        odom = Odometry()
        odom.header = self.generated_path.poses[index].header
        odom.pose.pose = self.generated_path.poses[index].pose
        odom.twist.twist = self.generated_path.twists[index]
        self.publish_desired.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
