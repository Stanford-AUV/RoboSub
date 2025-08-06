#!/usr/bin/env python3
import os
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R

from control.utils.state import State, Magnitude

ERROR_THRESHOLD = Magnitude(
    distance=0.1,        # m
    speed=0.1,           # m/s
    angle=0.1,           # rad
    angular_speed=0.1,   # rad/s
)

class PathTracker(Node):
    def __init__(self):
        super().__init__('path_tracker')

        # allow user to override JSON file on the command line
        self.declare_parameter(
            'waypoints_file',
            os.path.join(os.path.dirname(__file__), 'test_waypoints.json')
        )
        wp_file = self.get_parameter('waypoints_file').get_parameter_value().string_value

        # load the JSON into a nav_msgs/Path
        self.path = self._load_path_from_json(wp_file)
        n = len(self.path.poses)
        if n == 0:
            self.get_logger().error(f"No waypoints loaded from {wp_file}!")
            return
        self.get_logger().info(f"Loaded {n} waypoints from {wp_file}")

        # publisher for one‐by‐one Odometry‐waypoints
        self.waypoint_pub = self.create_publisher(Odometry, 'waypoint', 10)

        # tracking state
        self.index = 0
        self.last_wp_state = None

        # give time for connections, then publish first
        time.sleep(0.5)
        self._publish_current_waypoint()

        # now subscribe to odometry and drive the tracker
        self.create_subscription(
            Odometry, '/odometry/filtered', self._odom_cb, 10
        )

        self.count = 0

    def _load_path_from_json(self, json_path: str) -> Path:
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()

        try:
            with open(json_path, 'r') as f:
                data = json.load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to open {json_path}: {e}")
            return path

        for wp in data.get('waypoints', []):
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.header.stamp = self.get_clock().now().to_msg()
            # position
            pos = wp['position']
            ps.pose.position.x = pos['x']
            ps.pose.position.y = pos['y']
            ps.pose.position.z = pos['z']
            # orientation in roll/pitch/yaw (deg) → quaternion
            ori = wp['orientation']
            quat = R.from_euler(
                'xyz',
                [ori['roll'], ori['pitch'], ori['yaw']],
                degrees=True
            ).as_quat()  # returns [x,y,z,w]
            ps.pose.orientation.x = quat[0]
            ps.pose.orientation.y = quat[1]
            ps.pose.orientation.z = quat[2]
            ps.pose.orientation.w = quat[3]
            path.poses.append(ps)

        return path

    def _publish_current_waypoint(self):
        wp_msg = Odometry()
        wp_msg.header = self.path.header
        # copy the PoseStamped → Odometry.pose.pose
        wp_msg.pose.pose = self.path.poses[self.index].pose
        self.waypoint_pub.publish(wp_msg)

        # record for proximity checks
        self.last_wp_state = State.from_odometry_msg(wp_msg)
        self.get_logger().info(f"Published waypoint {self.index} {wp_msg.pose.pose}")

    def _odom_cb(self, odom: Odometry):
        if self.last_wp_state is None:
            return

        current = State.from_odometry_msg(odom)
        err = self.last_wp_state - current
        # err = current

        if self.count >= 10:
            self.get_logger().info(f"Current error: {err.magnitude()}")
            self.get_logger().info(f"Position {err.position}")
            self.get_logger().info(f"Velocity {err.velocity}")
            self.get_logger().info(f"Orientation {err.orientation}")
            self.get_logger().info(f"Angular Velocity {err.angular_velocity}")
            self.count = 0
        else:
            self.count += 1

        if err.magnitude() < ERROR_THRESHOLD:
            self.index += 1
            if self.index < len(self.path.poses):
                self.get_logger().info("Reached waypoint, advancing to next")
                self._publish_current_waypoint()
            else:
                self.get_logger().info("All waypoints reached!")
                # optionally: shut down or stop subscribing
                rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = PathTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
