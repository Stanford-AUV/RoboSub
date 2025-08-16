#!/usr/bin/env python3
import os
import json
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from msgs.msg import Waypoint
from std_msgs.msg import Bool

from control.utils.state import State, Magnitude

ERROR_THRESHOLD = Magnitude(
    distance=0.1,        # m
    speed=0.1,           # m/s
    angle=0.32,           # rad
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
        self.path, self.wp_meta = self._load_path_from_json(wp_file)
        n = len(self.wp_meta)
        if n == 0:
            self.get_logger().error(f"No waypoints loaded from {wp_file}!")
            return
        self.get_logger().info(f"Loaded {n} waypoints from {wp_file}")

        # publisher for one‐by‐one Odometry‐waypoints
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL   # latch last msg
        qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.waypoint_pub = self.create_publisher(Waypoint, 'waypoint', qos)
        self.object_reached = self.create_publisher(Bool, 'waypoint/object_reached', qos)

        # tracking state
        self.index = 0
        self.last_wp_state = None

        # give time for connections, then publish first
        # time.sleep(0.5)
        self._publish_current_waypoint()
        # self.create_timer(0.5, lambda: (self._publish_current_waypoint(), None))[0:0]

        # now subscribe to odometry and drive the tracker
        self.create_subscription(
            Odometry, '/odometry/filtered', self._odom_cb, 10
        )

        self.declare_parameter("stable_dwell_sec", 1.0)  # seconds inside band to be stable
        self.stable_dwell_sec = float(self.get_parameter("stable_dwell_sec").value)

        self.stable_start_ns = None   # when we entered the band
        self.stable_published = False # whether we've already sent True

        self.count = 0
        self.advance_timer = None

    def _advance_to_next(self):
        self.index += 1
        if self.index < len(self.wp_meta):
            self.get_logger().info("Advancing to next waypoint")
            self._publish_current_waypoint()
        else:
            self.get_logger().info("All waypoints reached!")
            rclpy.shutdown()

    def _load_path_from_json(self, json_path: str):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()

        wp_meta = []  # NEW

        try:
            with open(json_path, 'r') as f:
                data = json.load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to open {json_path}: {e}")
            return path, wp_meta  # CHANGED

        for wp in data.get('waypoints', []):
            purpose = (wp.get('purpose') or 'target').lower()
            subject = wp.get('subject', '')
            hold = float(wp.get('hold_time', 0.0))

            has_pose = 'position' in wp and 'orientation' in wp

            if has_pose:
                ps = PoseStamped()
                ps.header.frame_id = 'map'
                ps.header.stamp = self.get_clock().now().to_msg()

                pos = wp['position']
                ps.pose.position.x = pos['x']
                ps.pose.position.y = pos['y']
                ps.pose.position.z = pos['z']

                ori = wp['orientation']
                quat = R.from_euler('xyz', [ori['roll'], ori['pitch'], ori['yaw']], degrees=True).as_quat()
                ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = quat
                path.poses.append(ps)

                wp_meta.append({
                    'purpose': purpose,
                    'subject': subject,
                    'hold_time': hold,
                    'has_pose': True,
                    'path_index': len(path.poses) - 1
                })
            else:
                # Waypoint with no pose (e.g., "follow")
                wp_meta.append({
                    'purpose': purpose,
                    'subject': subject,
                    'hold_time': hold,
                    'has_pose': False,
                    'path_index': None
                })

        return path, wp_meta  # CHANGED


    def _publish_current_waypoint(self):
        # --- meta for this logical waypoint ---
        meta = self.wp_meta[self.index] if hasattr(self, 'wp_meta') else {
            'purpose': 'target',
            'subject': 'N/A',
            'hold_time': 0.0,
            'has_pose': True,
            'path_index': self.index
        }

        # Build outgoing message
        wp = Waypoint()
        wp.purpose = meta['purpose']
        wp.subject = meta.get('subject', '')
        wp.hold_time = float(meta.get('hold_time', 0.0))

        self.stable_start_ns = None
        self.stable_published = False
        self._pub_object_reached(False)

        # Warn if a FOLLOW waypoint is missing its subject
        if meta.get('purpose') == 'follow' and not wp.subject:
            self.get_logger().warn("FOLLOW waypoint missing 'subject'")

        # Default: no proximity gating unless we decide otherwise below
        self.last_wp_state = None

        if meta.get('has_pose', False):
            # Build an Odometry target from the corresponding pose in self.path
            pi = int(meta['path_index'])
            if pi < 0 or pi >= len(self.path.poses):
                self.get_logger().error(
                    f"Invalid path_index {pi} for waypoint {self.index}"
                )
                return
            odom_target = Odometry()
            odom_target.header = self.path.header
            odom_target.header.stamp = self.get_clock().now().to_msg()
            odom_target.pose.pose = self.path.poses[pi].pose
            wp.target = odom_target

            # Only "target" waypoints use proximity checks
            if wp.purpose.lower() == 'target':
                self.last_wp_state = State.from_odometry_msg(odom_target)

        # Publish the waypoint
        self.waypoint_pub.publish(wp)
        self.get_logger().info(
            f"Published waypoint {self.index} "
            f"(purpose={wp.purpose}, subject='{wp.subject}', has_pose={meta.get('has_pose', False)}, "
            f"hold_time={wp.hold_time:.2f}s)"
        )

        # --- Auto-advance handling ---
        # Cancel any prior timer
        if getattr(self, 'advance_timer', None) is not None:
            try:
                self.advance_timer.cancel()
            except Exception:
                pass
            self.advance_timer = None

        # If this waypoint has NO pose (e.g., "follow"), there's nothing to "reach".
        # Auto-advance after hold_time (or immediately if 0).
        if not meta.get('has_pose', False):
            delay = max(float(meta.get('hold_time', 0.0)), 0.01)
            def _on_timer():
                # one-shot
                if self.advance_timer is not None:
                    self.advance_timer.cancel()
                    self.advance_timer = None
                self._advance_to_next()
            self.advance_timer = self.create_timer(delay, _on_timer)

    
    def _pub_object_reached(self, value: bool):
        msg = Bool()
        msg.data = value
        self.object_reached.publish(msg)


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

        # if err.magnitude() < ERROR_THRESHOLD:
        #     self.count = 10
        #     self.get_logger().info("Reached waypoint, advancing to next")
        #     self._advance_to_next()

        inside = err.magnitude() < ERROR_THRESHOLD
        now_ns = self.get_clock().now().nanoseconds

        if inside:
            if self.stable_start_ns is None:
                # Just entered band
                self.stable_start_ns = now_ns
            else:
                elapsed_s = (now_ns - self.stable_start_ns) * 1e-9
                if (not self.stable_published) and elapsed_s >= self.stable_dwell_sec:
                    # Stable for long enough
                    self.stable_published = True
                    self._pub_object_reached(True)
                    self.get_logger().info(
                        f"Reached waypoint and stable for {self.stable_dwell_sec:.2f}s — advancing"
                    )
                    self._advance_to_next()
        else:
            # Outside band: reset timer and publish False if previously True
            if self.stable_published:
                self._pub_object_reached(False)
            self.stable_start_ns = None
            self.stable_published = False


def main(args=None):
    rclpy.init(args=args)
    node = PathTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
