"""Transforms camera-frame detections3d into a filtered odom-frame object goal.

Publishes /object/<id>/world_position (odom) for planning's go_to_object
pursuit, and /object/<id>/relative (base_link) as the live range readout.
Publishes only while detections are fresh, so subscribers can use "message
arriving" as "currently tracked".
"""

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from vision_msgs.msg import Detection3DArray

from perception.utils.object_world import (
    DetectionFilter,
    base_to_odom,
    camera_to_base,
)


class ObjectWorldLocalizer(Node):
    def __init__(self):
        super().__init__("object_world_localizer")

        self.declare_parameter("object_id", "gate")
        self.declare_parameter("min_score", 0.5)
        self.declare_parameter("cam_translation", [0.0, 0.0, 0.0])
        self.declare_parameter("cam_rpy_deg", [0.0, 0.0, 0.0])
        self.declare_parameter("filter_window", 5)
        self.declare_parameter("filter_alpha", 0.3)
        self.declare_parameter("max_jump_m", 3.0)
        self.declare_parameter("stale_sec", 2.0)
        self.declare_parameter("confirm_hits", 5)
        self.declare_parameter("confirm_window_sec", 2.0)
        self.declare_parameter("lock_radius_min_m", 0.05)
        self.declare_parameter("lock_radius_frac", 0.03)

        gp = self.get_parameter
        self.object_id = gp("object_id").get_parameter_value().string_value
        self.min_score = gp("min_score").get_parameter_value().double_value
        self.cam_translation = list(
            gp("cam_translation").get_parameter_value().double_array_value
        )
        self.cam_rpy_deg = list(
            gp("cam_rpy_deg").get_parameter_value().double_array_value
        )
        if len(self.cam_translation) != 3 or len(self.cam_rpy_deg) != 3:
            # Bad extrinsics would silently place the object in the wrong
            # spot; die loudly at startup instead.
            raise ValueError(
                "cam_translation and cam_rpy_deg must each have 3 elements"
            )
        self.filter = DetectionFilter(
            window=gp("filter_window").get_parameter_value().integer_value,
            alpha=gp("filter_alpha").get_parameter_value().double_value,
            max_jump_m=gp("max_jump_m").get_parameter_value().double_value,
            stale_sec=gp("stale_sec").get_parameter_value().double_value,
            confirm_hits=gp("confirm_hits").get_parameter_value().integer_value,
            confirm_window_sec=gp("confirm_window_sec").get_parameter_value().double_value,
            lock_radius_min_m=gp("lock_radius_min_m").get_parameter_value().double_value,
            lock_radius_frac=gp("lock_radius_frac").get_parameter_value().double_value,
        )

        self.odom_pos = None
        self.odom_quat = None
        self.last_relative = None
        self.was_publishing = False

        self.create_subscription(Odometry, "/odometry/filtered", self.on_odom, 10)
        self.create_subscription(
            Detection3DArray, "detections3d", self.on_detections, 10
        )
        self.pub_world = self.create_publisher(
            PointStamped, f"/object/{self.object_id}/world_position", 10
        )
        self.pub_relative = self.create_publisher(
            PointStamped, f"/object/{self.object_id}/relative", 10
        )
        self.create_timer(0.1, self.publish_goal)
        self.get_logger().info(
            f"Tracking '{self.object_id}' (min_score={self.min_score})"
        )

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.odom_pos = np.array([p.x, p.y, p.z])
        self.odom_quat = np.array([q.x, q.y, q.z, q.w])

    def on_detections(self, msg: Detection3DArray):
        if self.odom_pos is None:
            self.get_logger().warn(
                "Detections arriving but no /odometry/filtered yet",
                throttle_duration_sec=5.0,
            )
            return
        for det in msg.detections:
            if not det.results:
                continue
            hypo = det.results[0].hypothesis
            if hypo.class_id != self.object_id or hypo.score < self.min_score:
                continue
            c = det.bbox.center.position
            p_base = camera_to_base(
                [c.x, c.y, c.z], self.cam_translation, self.cam_rpy_deg
            )
            p_odom = base_to_odom(p_base, self.odom_pos, self.odom_quat)
            was_locked = self.filter.locked
            if self.filter.update(p_odom, self.now_sec(), self.odom_pos) is not None:
                self.last_relative = p_base
            if self.filter.locked and not was_locked:
                self.get_logger().info(
                    f"LOCKED '{self.object_id}' at odom "
                    f"[{p_odom[0]:.2f}, {p_odom[1]:.2f}, {p_odom[2]:.2f}]"
                )

    def publish_goal(self):
        t = self.now_sec()
        goal = self.filter.get(t)
        if goal is None:
            if self.was_publishing:
                self.was_publishing = False
                self.get_logger().warn(
                    f"Lock on '{self.object_id}' lost; re-confirming."
                )
            return
        self.was_publishing = True
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.point.x, msg.point.y, msg.point.z = goal.tolist()
        self.pub_world.publish(msg)

        if self.last_relative is not None:
            rel = PointStamped()
            rel.header.stamp = msg.header.stamp
            rel.header.frame_id = "base_link"
            rel.point.x, rel.point.y, rel.point.z = self.last_relative.tolist()
            self.pub_relative.publish(rel)
            self.get_logger().info(
                f"'{self.object_id}' at range "
                f"{np.linalg.norm(self.last_relative):.2f} m",
                throttle_duration_sec=2.0,
            )


def main(args=None):
    rclpy.init(args=args)
    node = ObjectWorldLocalizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
