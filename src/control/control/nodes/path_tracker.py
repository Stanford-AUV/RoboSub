from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from msgs.srv import GetWaypoints
import time

import numpy as np

from control.utils.state import State, Magnitude
import rclpy
from rclpy.node import Node

# class AngleVec(np.ndarray):
#     AXIS = {"roll": 0, "pitch":0, "yaw":0}


ERROR_THRESHOLD = Magnitude(
    distance=0.1,  # m
    speed=0.1,  # m/s
    # angle=0.1,  # radians
    # angle=(0.2, 0.2, 0.05),
    angle=0.0,
    angular_speed=0.1,  # radians/s
)

ANGLE_ERROR_THRESHOLD = {"pitch": 0.2, "roll": 0.2, "yaw": 0.05}


class PathTracker(Node):
    """
    Node that determines the next waypoint on the path to follow for the controller node.
    """

    def __init__(self):
        super().__init__("path_tracker")
        self.get_logger().info("he")

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

        time.sleep(1)
        self.publish_waypoint()

        self.odometry_subscription = self.create_subscription(
            Odometry, "/odometry/filtered", self.odometry_callback, 10
        )

        self.msg_cnt = 0

    def set_waypoints(self):
        req = GetWaypoints.Request()
        future = self.waypoints_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.path = future.result().waypoints
        self.get_logger().info(f"Got {len(self.path.poses)} waypoints")

    def odometry_callback(self, robot: Odometry):
        self.get_logger().info("whee")
        robot_odom = State.from_odometry_msg(robot)
        error = self.last_waypoint - robot_odom
        error_magnitude = error.magnitude()
        import tf_transformations

        if self.msg_cnt >= 20:
            self.get_logger().info(f"Odom pos is {robot.pose.pose.position}")
            self.get_logger().info(
                f"Odom ori (r,p,y) is {tf_transformations.euler_from_quaternion([robot.pose.pose.orientation.x, robot.pose.pose.orientation.y, robot.pose.pose.orientation.z, robot.pose.pose.orientation.w])}"
            )
            self.get_logger().info(f"Error is {error_magnitude}")
            self.msg_cnt = 0
        else:
            self.msg_cnt += 1

        err_angle = error_magnitude.angle
        # self.get_logger().info(f"rioguh7f {ERROR_THRESHOLD.angle["roll"]}")
        # roll_err = err_angle[0]
        # pitch_err = err_angle[1]
        # yaw_err = err_angle[2]
        roll_err, pitch_err, yaw_err = error.orientation.rpy(order='xyz')
        # self.get_logger().info(f"error: {roll_err}, {pitch_err}")

        if (
            error_magnitude.distance < ERROR_THRESHOLD.distance
            and error_magnitude.speed < ERROR_THRESHOLD.speed
            # and abs(roll_err) < ERROR_THRESHOLD.angle["roll"]
            # and abs(pitch_err) < ERROR_THRESHOLD.angle["pitch"]
            # and abs(yaw_err) < ERROR_THRESHOLD.angle["yaw"]
            and roll_err < ANGLE_ERROR_THRESHOLD["roll"]
            and pitch_err < ANGLE_ERROR_THRESHOLD["pitch"]
            and yaw_err < ANGLE_ERROR_THRESHOLD["yaw"]
            and error_magnitude.angular_speed < ERROR_THRESHOLD.angular_speed
        ):
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
    print(";;;;", flush=True)
    rclpy.init(args=args)
    path_tracker = PathTracker()
    rclpy.spin(path_tracker)
    path_tracker.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
