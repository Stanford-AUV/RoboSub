from nav_msgs.msg import Odometry
from msgs.msg import GeneratedPath


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

        self.waypoint_publisher = self.create_publisher(Odometry, "waypoint", 10)

        self.path_index = 0
        self.generated_path: GeneratedPath | None = None

        self.timer = self.create_timer(0.1, self.next_waypoint)

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
        waypoint.twist.twist = self.generated_path.twists[self.path_index]

        self.get_logger().info(f"Publishing waypoint for {self.path_index}")
        self.waypoint_publisher.publish(waypoint)

        self.path_index += 1


def main(args=None):
    rclpy.init(args=args)
    path_tracker = PathTracker()
    rclpy.spin(path_tracker)
    path_tracker.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
