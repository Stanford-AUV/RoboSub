from nav_msgs.msg import Odometry
from msgs.msg import GeneratedPath

import math
import rclpy
from rclpy.node import Node


class PathTracker(Node):
    """
    Node that determines the next waypoint on the path to follow for the controller node.
    """

    def __init__(self):
        super().__init__("path_tracker")

        self.path_subscription = self.create_subscription(
            GeneratedPath, "generated_path", self.path_callback, 10
        )

        self.state_subscription = self.create_subscription(
            Odometry, "odometry", self.state_callback, 10
        )

        self.waypoint_publisher = self.create_publisher(Odometry, "waypoint", 10)

        self.path_index = 0
        self.generated_path: GeneratedPath | None = None
        self.cur_state: Odometry | None = None
        self.last_waypoint: Odometry | None = None

    def state_callback(self, msg: Odometry):
        self.cur_state = msg
        waypoint = self.last_waypoint
        if waypoint is None:
            self.next_waypoint()
            return

        # Calculate distance between current position and waypoint
        current_pos = self.cur_state.pose.pose.position
        waypoint_pos = waypoint.pose.pose.position
        dx = current_pos.x - waypoint_pos.x
        dy = current_pos.y - waypoint_pos.y
        dz = current_pos.z - waypoint_pos.z
        distance = (dx**2 + dy**2 + dz**2) ** 0.5

        # Calculate orientation difference
        current_orient = self.cur_state.pose.pose.orientation
        waypoint_orient = waypoint.pose.pose.orientation
        dot = (
            current_orient.x * waypoint_orient.x
            + current_orient.y * waypoint_orient.y
            + current_orient.z * waypoint_orient.z
            + current_orient.w * waypoint_orient.w
        )
        # Clamp the dot product to avoid domain errors in acos
        dot = max(min(dot, 1.0), -1.0)
        angle_diff = 2.0 * math.acos(dot)

        # If within 0.1 meter threshold *and* orientation difference < 0.1 rad, advance to next waypoint
        # self.get_logger().info(f"Distance: " + str(distance))
        # self.get_logger().info(f"Angle diff: " + str(angle_diff))
        if distance < 0.1 and angle_diff < 0.2:
            self.next_waypoint()

    def path_callback(self, msg: GeneratedPath):
        self.generated_path = msg

    def next_waypoint(self):
        if self.generated_path is None:
            return

        if self.path_index >= len(self.generated_path.poses):
            return

        waypoint = Odometry()
        waypoint.header = self.generated_path.header
        waypoint.pose.pose = self.generated_path.poses[self.path_index].pose
        # waypoint.twist.twist = self.generated_path.twists[self.path_index]

        self.get_logger().info(f"Publishing waypoint for {self.path_index}")
        self.waypoint_publisher.publish(waypoint)
        self.last_waypoint = waypoint

        self.path_index += 1


def main(args=None):
    rclpy.init(args=args)
    path_tracker = PathTracker()
    rclpy.spin(path_tracker)
    path_tracker.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
