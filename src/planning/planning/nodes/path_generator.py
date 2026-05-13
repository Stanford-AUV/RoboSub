import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from msgs.msg import GeneratedPath  # Custom message import
from rclpy import Parameter

import numpy as np
from scipy.spatial.transform import Rotation

from planning.utils.create_path import create_path

class PathGenerator(Node):
    def __init__(self):
        super().__init__("path_generator")


        self.waypoints_subscriber = self.create_subscription(
            Path, "/waypoints", self.waypoints_callback, 10
        )

        self.publish_desired = self.create_publisher(Odometry, "/desired/pose", 10)

        self.generated_path = None
        self.path_start_time = None

        self.create_timer(1.0 / 60.0, self.publish_pose) 

        self.get_logger().info("PathGenerator node has been started.")

    def waypoints_callback(self, msg: Path):
        self.get_logger().info("Received waypoints, generating path...")

        x_positions = []
        y_positions = []
        z_positions = []
        roll_angles = []
        pitch_angles = []
        yaw_angles = []

        
        for pose_stamped in msg.poses:
            x_positions.append(pose_stamped.pose.position.x)
            y_positions.append(pose_stamped.pose.position.y)
            z_positions.append(pose_stamped.pose.position.z)

            orientation_q = pose_stamped.pose.orientation
            quaternion = [
                orientation_q.x,
                orientation_q.y,
                orientation_q.z,
                orientation_q.w,
            ]

            euler = Rotation.from_quat(quaternion).as_euler("xyz", degrees=True)
            roll_angles.append(euler[0]) 
            pitch_angles.append(euler[1])
            yaw_angles.append(euler[2]) 

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
                x_positions, y_positions, z_positions, roll_angles, pitch_angles, yaw_angles
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
