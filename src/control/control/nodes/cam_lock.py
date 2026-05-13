#!/usr/bin/env python3

import math
import time
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

from vision_msgs.msg import Detection3DArray
from tf_transformations import euler_from_quaternion, quaternion_from_euler

from msgs.msg import Waypoint


class CamLock(Node):
    """
    Subscribe to 3D detections and publish Waypoint targets that move the robot
    so the chosen object class is centered in the camera and kept at a fixed standoff.

    Coordinates:
      - Assumes detection bbox.center.position (x,y,z) is in the CAMERA frame (meters),
        with x right, y down, z forward (DepthAI / OpenCV convention).
      - We compute a correction vector in camera frame and rotate it into the world
        using the *current yaw* (and an optional camera_yaw_in_base offset).
    """

    def __init__(self):
        super().__init__("cam_lock")

        # ---- Parameters ----
        self.declare_parameter("subject", "shark")        # class label to lock on
        self.declare_parameter("stand_off", 0.5)           # meters from camera to target (desired z)
        self.declare_parameter("camera_yaw_in_base_deg", 0.0)  # camera yaw offset wrt robot base (deg)
        self.declare_parameter("publish_hold_time", 0.0)   # seconds
        self.declare_parameter("min_publish_period", 0.2)  # throttle (s)

        self.subject: str = self.get_parameter("subject").get_parameter_value().string_value
        self.stand_off: float = self.get_parameter("stand_off").get_parameter_value().double_value
        self.cam_yaw_off_deg: float = self.get_parameter("camera_yaw_in_base_deg").get_parameter_value().double_value
        self.hold_time: float = self.get_parameter("publish_hold_time").get_parameter_value().double_value
        self.min_period: float = self.get_parameter("min_publish_period").get_parameter_value().double_value

        # Allow runtime changes via ROS params
        self.add_on_set_parameters_callback(self._on_param_update)

        # ---- State ----
        self._last_odom: Optional[Odometry] = Odometry()#None CHANGE LATER PLSL!!
        self._last_pub_time = 0.0

        # ---- I/O ----
        # Subscribe to robot odometry
        self.create_subscription(Odometry, "/odometry/filtered", self._odom_cb, 10)

        # Subscribe to 3D detections
        self.create_subscription(Detection3DArray, "detections3d", self._detections_cb, 10)

        # Publish Waypoint (latching/TRANSIENT_LOCAL so the latest target is retained)
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        self._wp_pub = self.create_publisher(Waypoint, "/cam_lock/waypoint", qos)

        self.get_logger().info(
            f"cam_lock running | subject='{self.subject}' stand_off={self.stand_off:.2f} m "
            f"cam_yaw_off={self.cam_yaw_off_deg:.1f} deg"
        )

    # ----- Public getters/setters if you want to change subject programmatically -----
    def set_subject(self, label: str):
        self.subject = label

    def get_subject(self) -> str:
        return self.subject

    # ----- Callbacks -----
    def _on_param_update(self, params):
        for p in params:
            if p.name == "subject" and p.value is not None:
                self.subject = str(p.value)
            elif p.name == "stand_off" and p.value is not None:
                self.stand_off = float(p.value)
            elif p.name == "camera_yaw_in_base_deg" and p.value is not None:
                self.cam_yaw_off_deg = float(p.value)
            elif p.name == "publish_hold_time" and p.value is not None:
                self.hold_time = float(p.value)
            elif p.name == "min_publish_period" and p.value is not None:
                self.min_period = float(p.value)
        return rclpy.parameter.SetParametersResult(successful=True)

    def _odom_cb(self, msg: Odometry):
        self._last_odom = msg

    def _detections_cb(self, msg: Detection3DArray):
        # Need current odometry to publish an absolute target pose
        if self._last_odom is None:
            return

        now = time.time()
        if now - self._last_pub_time < self.min_period:
            return  # throttle

        # Find detections whose class_id matches the subject (case-insensitive)
        target_label = (self.subject or "").lower()
        candidates = []
        for det in msg.detections:
            for res in det.results:
                label = (res.hypothesis.class_id or "").lower()
                if label == target_label:
                    # Prefer the closest one (smallest z)
                    z = det.bbox.center.position.z
                    score = float(res.hypothesis.score)
                    candidates.append((z, -score, det))  # z asc, score desc

        if not candidates:
            return

        candidates.sort()
        _, _, best = candidates[0]
        self.get_logger().info(f"hey {candidates}")

        # Get detection center in CAMERA frame (meters)
        cx = best.bbox.center.position.x  # +x right
        cy = best.bbox.center.position.y  # +y down
        cz = best.bbox.center.position.z  # +z forward

        # Compute correction in camera frame to make (x,y)->0 and z->stand_off
        # Move camera by (-cx, -cy, stand_off - cz)
        delta_cam = np.array([-cx, -cy, self.stand_off - cz], dtype=float)

        # Rotate delta into world/odom frame using current yaw and camera yaw offset
        # Get current yaw from odometry orientation
        q = self._last_odom.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        yaw_total = yaw + math.radians(self.cam_yaw_off_deg)

        c, s = math.cos(yaw_total), math.sin(yaw_total)
        Rz = np.array([[c, -s, 0.0],
                       [s,  c, 0.0],
                       [0.0, 0.0, 1.0]], dtype=float)
        delta_world = Rz @ delta_cam

        # Desired absolute position = current position + delta_world
        cur = self._last_odom.pose.pose
        goal_pos = np.array([cur.position.x, cur.position.y, cur.position.z], dtype=float) + delta_world

        # Keep current orientation (you could also point toward the target if desired)
        goal_q = [cur.orientation.x, cur.orientation.y, cur.orientation.z, cur.orientation.w]

        # Build Odometry target in the same frame as /odometry/filtered
        target = Odometry()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = self._last_odom.header.frame_id or "map"
        target.child_frame_id = self._last_odom.child_frame_id

        target.pose.pose.position.x = float(goal_pos[0])
        target.pose.pose.position.y = float(goal_pos[1])
        target.pose.pose.position.z = float(goal_pos[2])

        target.pose.pose.orientation.x = goal_q[0]
        target.pose.pose.orientation.y = goal_q[1]
        target.pose.pose.orientation.z = goal_q[2]
        target.pose.pose.orientation.w = goal_q[3]

        # Publish as a Waypoint (purpose="target")
        wp = Waypoint()
        wp.purpose = "follow"
        wp.subject = self.subject
        wp.target = target
        wp.hold_time = float(self.hold_time)
        self._wp_pub.publish(wp)

        self._last_pub_time = now
        self.get_logger().info(
            f"cam_lock: locked '{self.subject}' | desired Δ_cam={delta_cam.round(3)} m "
            f"→ goal=({goal_pos[0]:.2f},{goal_pos[1]:.2f},{goal_pos[2]:.2f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = CamLock()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
