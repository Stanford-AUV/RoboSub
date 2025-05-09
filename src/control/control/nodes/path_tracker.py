from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from msgs.srv import GetWaypoints

import numpy as np

from control.utils.state import State, Magnitude
import rclpy
from rclpy.node import Node

ERROR_THRESHOLD = Magnitude(
    distance=0.1,  # m
    speed=2,  # m/s
    angle=0.5,  # radians
    angular_speed=0.1,  # radians/s
)


class PathTracker(Node):
    """
    Node that determines the next waypoint on the path to follow for the controller node.
    """

    def __init__(self):
        super().__init__("path_tracker")

        self.waypoint_publisher = self.create_publisher(Odometry, "waypoint", 10)

        self.path_index = 0
        self.path: Path | None = None
        self.last_waypoint: Odometry | None = None

        self.waypoints_cli = self.create_client(GetWaypoints, "get_waypoints")
        while not self.waypoints_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for waypoints service...")

        self.set_waypoints()
        if len(self.path.poses) == 0:
            self.get_logger().error("No waypoints found")
            return

        self.publish_waypoint()

        self.odometry_subscription = self.create_subscription(
            Odometry, "/odometry/filtered", self.odometry_callback, 10
        )

    def set_waypoints(self):
        req = GetWaypoints.Request()
        future = self.waypoints_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.path = future.result().waypoints
        self.get_logger().info(f"Got {len(self.path.poses)} waypoints")

    def odometry_callback(self, robot: Odometry):
        robot_odom = State.from_odometry_msg(robot)
        error = self.last_waypoint - robot_odom
        error_magnitude = error.magnitude()
        if error_magnitude < ERROR_THRESHOLD:
            self.path_index += 1
            if self.path_index >= len(self.path.poses):
                self.get_logger().info("Reached end of path")
            else:
                self.get_logger().info("Next waypoint")
                self.publish_waypoint()

    def publish_waypoint(self):
        assert self.path is not None, "Path is not set"
        assert self.path_index < len(
            self.path.poses
        ), f"Path index {self.path_index} is out of bounds for path length {len(self.path.poses)}"

        waypoint = Odometry()
        waypoint.header = self.path.header
        waypoint.pose.pose = self.path.poses[self.path_index].pose
        # waypoint.twist.twist = self.path.twists[self.path_index]

        self.get_logger().info(f"Publishing waypoint for {self.path_index}")
        self.waypoint_publisher.publish(waypoint)

        self.last_waypoint = State.from_odometry_msg(waypoint)


def main(args=None):
    rclpy.init(args=args)
    path_tracker = PathTracker()
    rclpy.spin(path_tracker)
    path_tracker.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
