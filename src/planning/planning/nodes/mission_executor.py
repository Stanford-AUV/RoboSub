"""
mission_executor.py
-------------------
Reads a mission YAML and executes tasks sequentially by publishing
nav_msgs/Path messages to /waypoints (consumed by path_generator.py).

Supported task types:
  gate     - waits for PoseStamped on a perception topic, then generates
             approach -> (style roll) -> pass-through waypoints offset to
             the correct side based on /perception/gate/side.
  waypoint - publishes a single fixed world-frame waypoint immediately.
"""

import os
import math
import yaml

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from scipy.spatial.transform import Rotation
import numpy as np

COMPLETION_BUFFER_S   = 2.0
TICK_RATE_S           = 0.5
DEFAULT_SIDE_OFFSET_M = 0.6


def euler_to_quat(roll_deg, pitch_deg, yaw_deg):
    return Rotation.from_euler(
        "xyz", [roll_deg, pitch_deg, yaw_deg], degrees=True
    ).as_quat()


def make_pose_stamped(x, y, z, roll_deg, pitch_deg, yaw_deg, stamp, frame_id="map"):
    ps = PoseStamped()
    ps.header.stamp    = stamp
    ps.header.frame_id = frame_id
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.position.z = float(z)
    q = euler_to_quat(roll_deg, pitch_deg, yaw_deg)
    ps.pose.orientation.x = q[0]
    ps.pose.orientation.y = q[1]
    ps.pose.orientation.z = q[2]
    ps.pose.orientation.w = q[3]
    return ps


def gate_waypoints(gate_pose, side_info, task_cfg, stamp):
    """
    Build [approach, mid-gate (with optional roll), exit] waypoints.

    side_info from /perception/gate/side:
      "survey_on_right" | "survey_on_left" |
      "survey_repair"   | "search_rescue"  | "unknown"
    """
    depth          = float(task_cfg.get("depth", -1.5))
    approach_dist  = float(task_cfg.get("approach_distance", 2.0))
    pass_dist      = float(task_cfg.get("pass_distance", 2.0))
    preferred_role = task_cfg.get("preferred_role", "survey_repair")
    style_roll     = bool(task_cfg.get("style_roll", False))
    style_roll_deg = float(task_cfg.get("style_roll_deg", 90.0))
    side_offset    = float(task_cfg.get("side_offset_m", DEFAULT_SIDE_OFFSET_M))

    pos = gate_pose.pose.position
    ori = gate_pose.pose.orientation
    rot = Rotation.from_quat([ori.x, ori.y, ori.z, ori.w])

    forward_h = np.array(rot.apply([1.0, 0.0, 0.0])[:2] + [0.0])
    right_h   = np.array(rot.apply([0.0, -1.0, 0.0])[:2] + [0.0])
    for v in (forward_h, right_h):
        n = np.linalg.norm(v)
        if n > 1e-6:
            v /= n

    gx, gy  = pos.x, pos.y
    yaw_deg = math.degrees(math.atan2(forward_h[1], forward_h[0]))

    # Decide which side to offset toward
    want_survey = (preferred_role == "survey_repair")
    if side_info in ("survey_repair", "search_rescue"):
        go_right = (side_info == preferred_role)
    elif side_info == "survey_on_right":
        go_right = want_survey
    elif side_info == "survey_on_left":
        go_right = not want_survey
    else:
        go_right = False  # centre

    lateral = right_h * side_offset if go_right else -right_h * side_offset

    approach_wp = make_pose_stamped(
        gx - forward_h[0] * approach_dist + lateral[0],
        gy - forward_h[1] * approach_dist + lateral[1],
        depth, 0.0, 0.0, yaw_deg, stamp,
    )
    mid_roll = style_roll_deg if style_roll else 0.0
    mid_wp   = make_pose_stamped(
        gx + lateral[0], gy + lateral[1],
        depth, mid_roll, 0.0, yaw_deg, stamp,
    )
    exit_wp  = make_pose_stamped(
        gx + forward_h[0] * pass_dist + lateral[0],
        gy + forward_h[1] * pass_dist + lateral[1],
        depth, 0.0, 0.0, yaw_deg, stamp,
    )
    return [approach_wp, mid_wp, exit_wp]


class MissionExecutor(Node):

    def __init__(self, mission_yaml_path: str):
        super().__init__("mission_executor")

        self.waypoints_pub = self.create_publisher(Path, "/waypoints", 10)

        self.latest_path_duration = None
        self.path_start_time      = None

        self.tasks      = self._load_mission(mission_yaml_path)
        self.task_index = 0

        self._waiting_for_perception  = False
        self._perception_sub          = None
        self._side_sub                = None
        self._gate_pose               = None
        self._gate_side               = None
        self._perception_deadline     = None

        self.create_timer(TICK_RATE_S, self._tick)
        self.get_logger().info(
            f"MissionExecutor ready — {len(self.tasks)} task(s) loaded."
        )

    def _load_mission(self, path):
        with open(path, "r") as f:
            data = yaml.safe_load(f)
        tasks = data.get("mission", [])
        self.get_logger().info(
            f"Mission: {[t.get('label', t['task']) for t in tasks]}"
        )
        return tasks

    # --- perception callbacks ---

    def _gate_pose_cb(self, msg: PoseStamped):
        if not self._waiting_for_perception:
            return
        self._gate_pose = msg
        self._maybe_execute_gate()

    def _gate_side_cb(self, msg: String):
        if not self._waiting_for_perception:
            return
        self._gate_side = msg.data
        self._maybe_execute_gate()

    def _maybe_execute_gate(self):
        if self._gate_pose is None:
            return
        side  = self._gate_side if self._gate_side is not None else "unknown"
        task  = self.tasks[self.task_index]
        label = task.get("label", "gate")

        self.get_logger().info(
            f"[{label}] Gate detected. Side={side}  "
            f"Preferred={task.get('preferred_role','survey_repair')}"
        )
        self._waiting_for_perception = False
        self._teardown_perception_subs()

        poses = gate_waypoints(self._gate_pose, side, task,
                               self.get_clock().now().to_msg())
        self._publish_waypoints(poses)
        self._gate_pose = None
        self._gate_side = None

    def _teardown_perception_subs(self):
        if self._perception_sub:
            self.destroy_subscription(self._perception_sub)
            self._perception_sub = None
        if self._side_sub:
            self.destroy_subscription(self._side_sub)
            self._side_sub = None

    # --- state machine ---

    def _tick(self):
        if self.task_index >= len(self.tasks):
            return

        task = self.tasks[self.task_index]

        if self._waiting_for_perception:
            if self.get_clock().now() >= self._perception_deadline:
                self.get_logger().warn(
                    f"[{task.get('label','gate')}] Perception timeout — skipping."
                )
                self._waiting_for_perception = False
                self._teardown_perception_subs()
                self._gate_pose = None
                self._gate_side = None
                self._advance()
            return

        if self.path_start_time is not None and self.latest_path_duration is not None:
            elapsed = (self.get_clock().now() - self.path_start_time).nanoseconds / 1e9
            if elapsed >= self.latest_path_duration + COMPLETION_BUFFER_S:
                self._advance()
                return

        if self.path_start_time is None:
            self._start_task(task)

    def _start_task(self, task):
        task_type = task["task"]
        label     = task.get("label", task_type)

        if task_type == "gate":
            timeout = float(task.get("perception_timeout", 30.0))
            self._perception_deadline = self.get_clock().now() + Duration(
                seconds=int(timeout),
                nanoseconds=int((timeout % 1) * 1e9),
            )
            self._waiting_for_perception = True
            pose_topic = task.get("perception_topic",      "/perception/gate/pose")
            side_topic = task.get("perception_side_topic", "/perception/gate/side")
            self._perception_sub = self.create_subscription(
                PoseStamped, pose_topic, self._gate_pose_cb, 10
            )
            self._side_sub = self.create_subscription(
                String, side_topic, self._gate_side_cb, 10
            )
            self.get_logger().info(
                f"[{label}] Waiting for gate perception (timeout {timeout:.0f}s)..."
            )

        elif task_type == "waypoint":
            pose = make_pose_stamped(
                task.get("x", 0.0), task.get("y", 0.0), task.get("z", 0.0),
                task.get("roll", 0.0), task.get("pitch", 0.0), task.get("yaw", 0.0),
                self.get_clock().now().to_msg(),
            )
            self.get_logger().info(f"[{label}] Publishing fixed waypoint.")
            self._publish_waypoints([pose])

        else:
            self.get_logger().warn(f"Unknown task type '{task_type}' — skipping.")
            self._advance()

    def _publish_waypoints(self, poses):
        path_msg              = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        path_msg.poses        = poses
        self.waypoints_pub.publish(path_msg)

        total_dist = sum(
            math.dist(
                [poses[i-1].pose.position.x, poses[i-1].pose.position.y, poses[i-1].pose.position.z],
                [poses[i].pose.position.x,   poses[i].pose.position.y,   poses[i].pose.position.z],
            )
            for i in range(1, len(poses))
        )
        self.latest_path_duration = max(total_dist / 1.0, 2.0)
        self.path_start_time      = self.get_clock().now()
        self.get_logger().info(
            f"Published {len(poses)} waypoint(s) — "
            f"est. duration {self.latest_path_duration:.1f}s."
        )

    def _advance(self):
        self.task_index          += 1
        self.path_start_time      = None
        self.latest_path_duration = None
        if self.task_index >= len(self.tasks):
            self.get_logger().info("Mission complete.")
        else:
            nxt = self.tasks[self.task_index]
            self.get_logger().info(
                f"-> Task {self.task_index}: {nxt['task']} ({nxt.get('label','')})"
            )


def main(args=None):
    rclpy.init(args=args)
    SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
    yaml_path  = os.path.join(SCRIPT_DIR, "..", "mission.yaml")
    node = MissionExecutor(yaml_path)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
