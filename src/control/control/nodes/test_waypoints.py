import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from msgs.srv import GetWaypoints

import os
import json
import yaml
import numpy as np
from scipy.spatial.transform import Rotation


def wrap_pi(rad: float) -> float:
    return float((rad + np.pi) % (2 * np.pi) - np.pi)


class WaypointTest(Node):
    def __init__(self):
        super().__init__("test_waypoints")
        self.get_logger().info("WaypointTest node has been started.")

        # Optional: keep service (does NOT publish /desired/pose)
        self.waypoints_srv = self.create_service(GetWaypoints, "get_waypoints", self.get_waypoints)

        # The ONLY desired publisher
        self.desired_pub = self.create_publisher(Odometry, "/desired/pose", 10)

        # Current odom
        self.curr_odom: Odometry | None = None
        self.curr_sub = self.create_subscription(Odometry, "/odometry/filtered", self._on_curr_odom, 10)

        # Paths
        script_dir = os.path.dirname(os.path.realpath(__file__))
        self.json_path = os.path.join(script_dir, "test_waypoints.json")
        self.yaml_path = os.path.abspath(os.path.join(script_dir, "..", "config", "threshold.yaml"))

        # Load waypoints JSON
        self.data = self._load_json(self.json_path)
        self.waypoints = self.data.get("waypoints", [])
        if not self.waypoints:
            self.get_logger().error(f"No waypoints found in {self.json_path}")
            self.waypoints = []

        # Load thresholds YAML
        thr = self._load_thresholds_yaml(self.yaml_path)

        # Thresholds (YAML, with defaults)
        self.pos_thr = {
            "x": float(thr.get("pos", {}).get("x", 0.10)),
            "y": float(thr.get("pos", {}).get("y", 0.10)),
            "z": float(thr.get("pos", {}).get("z", 0.10)),
        }
        self.rpy_thr = {
            "roll": np.deg2rad(float(thr.get("rpy_deg", {}).get("roll", 5.0))),
            "pitch": np.deg2rad(float(thr.get("rpy_deg", {}).get("pitch", 5.0))),
            "yaw": np.deg2rad(float(thr.get("rpy_deg", {}).get("yaw", 5.0))),
        }

        self.get_logger().info(f"{self.rpy_thr}")
        self.vel_thr = {
            "x": float(thr.get("vel", {}).get("x", 0.05)),
            "y": float(thr.get("vel", {}).get("y", 0.05)),
            "z": float(thr.get("vel", {}).get("z", 0.05)),
        }
        self.ang_vel_thr = {
            "x": float(thr.get("ang_vel", {}).get("x", 0.05)),
            "y": float(thr.get("ang_vel", {}).get("y", 0.05)),
            "z": float(thr.get("ang_vel", {}).get("z", 0.05)),
        }

        self.dwell_s = float(thr.get("dwell_s", 0.5))
        self.cooldown_s = float(thr.get("cooldown_s", 0.75))
        self.publish_hz = float(thr.get("publish_hz", 10.0))

        # Sequencing
        self.wp_idx = 0
        self._inside_since: float | None = None
        self._cooldown_until = 0.0

        self.timer = self.create_timer(1.0 / self.publish_hz, self._tick)

        self.get_logger().info(
            "Loaded thresholds from: %s" % self.yaml_path
        )
        self.get_logger().info(
            "Thresholds: "
            f"pos={self.pos_thr} m, "
            f"rpy(rad)={ {k: float(v) for k,v in self.rpy_thr.items()} }, "
            f"vel={self.vel_thr} m/s, "
            f"ang_vel={self.ang_vel_thr} rad/s, "
            f"dwell={self.dwell_s}s, cooldown={self.cooldown_s}s, "
            f"publish_hz={self.publish_hz}"
        )

    # Service can be used by other nodes, but does not publish desired
    def get_waypoints(self, request: GetWaypoints.Request, response: GetWaypoints.Response):
        return response

    def _on_curr_odom(self, msg: Odometry):
        self.curr_odom = msg

    def _load_json(self, path: str) -> dict:
        with open(path, "r") as f:
            return json.load(f)

    def _load_thresholds_yaml(self, path: str) -> dict:
        if not os.path.exists(path):
            self.get_logger().warn(f"threshold.yaml not found at {path}; using defaults")
            return {}
        try:
            with open(path, "r") as f:
                data = yaml.safe_load(f)
            return data if isinstance(data, dict) else {}
        except Exception as e:
            self.get_logger().error(f"Failed to read thresholds yaml {path}: {e}; using defaults")
            return {}

    def _get_waypoint_target(self, i: int) -> dict:
        wp = self.waypoints[i]
        pos = wp.get("position", {})
        ori = wp.get("orientation", {})
        vel = wp.get("velocity", {})  # optional
        ang = wp.get("angular_velocity", {})  # optional

        return {
            "pos": {
                "x": float(pos.get("x", 0.0)),
                "y": float(pos.get("y", 0.0)),
                "z": float(pos.get("z", 0.0)),
            },
            # roll/pitch/yaw in degrees in JSON
            "rpy_rad": {
                "roll": np.deg2rad(float(ori.get("roll", 0.0))),
                "pitch": np.deg2rad(float(ori.get("pitch", 0.0))),
                "yaw": np.deg2rad(float(ori.get("yaw", 0.0))),
            },
            "vel": {
                "x": float(vel.get("x", 0.0)),
                "y": float(vel.get("y", 0.0)),
                "z": float(vel.get("z", 0.0)),
            },
            "ang_vel": {
                "x": float(ang.get("x", 0.0)),
                "y": float(ang.get("y", 0.0)),
                "z": float(ang.get("z", 0.0)),
            },
        }

    def _publish_desired_odom(self, tgt: dict):
        des = Odometry()
        des.header.stamp = self.get_clock().now().to_msg()
        des.header.frame_id = "map"  # adjust if your system expects odom/map

        des.pose.pose.position.x = tgt["pos"]["x"]
        des.pose.pose.position.y = tgt["pos"]["y"]
        des.pose.pose.position.z = tgt["pos"]["z"]

        q = Rotation.from_euler(
            "xyz",
            [tgt["rpy_rad"]["roll"], tgt["rpy_rad"]["pitch"], tgt["rpy_rad"]["yaw"]],
            degrees=False,
        ).as_quat()
        des.pose.pose.orientation.x = float(q[0])
        des.pose.pose.orientation.y = float(q[1])
        des.pose.pose.orientation.z = float(q[2])
        des.pose.pose.orientation.w = float(q[3])

        des.twist.twist.linear.x = tgt["vel"]["x"]
        des.twist.twist.linear.y = tgt["vel"]["y"]
        des.twist.twist.linear.z = tgt["vel"]["z"]
        des.twist.twist.angular.x = tgt["ang_vel"]["x"]
        des.twist.twist.angular.y = tgt["ang_vel"]["y"]
        des.twist.twist.angular.z = tgt["ang_vel"]["z"]

        self.desired_pub.publish(des)

    def _tick(self):
        if not self.waypoints:
            return

        # clamp index (hold last)
        if self.wp_idx >= len(self.waypoints):
            self.wp_idx = len(self.waypoints) - 1

        tgt = self._get_waypoint_target(self.wp_idx)
        self._publish_desired_odom(tgt)

        if self.curr_odom is None:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        if now < self._cooldown_until:
            return

        p = self.curr_odom.pose.pose.position
        qmsg = self.curr_odom.pose.pose.orientation
        v = self.curr_odom.twist.twist.linear
        w = self.curr_odom.twist.twist.angular

        # Position component errors
        ex = p.x - tgt["pos"]["x"]
        ey = p.y - tgt["pos"]["y"]
        ez = p.z - tgt["pos"]["z"]

        # Orientation component errors (roll,pitch,yaw), wrapped
        r_curr = Rotation.from_quat([qmsg.x, qmsg.y, qmsg.z, qmsg.w])
        roll_c, pitch_c, yaw_c = r_curr.as_euler("xyz", degrees=False)
        eroll = wrap_pi(roll_c - tgt["rpy_rad"]["roll"])
        epitch = wrap_pi(pitch_c - tgt["rpy_rad"]["pitch"])
        eyaw = wrap_pi(yaw_c - tgt["rpy_rad"]["yaw"])

        # Velocity component errors
        evx = v.x - tgt["vel"]["x"]
        evy = v.y - tgt["vel"]["y"]
        evz = v.z - tgt["vel"]["z"]

        # Angular velocity component errors
        ewx = w.x - tgt["ang_vel"]["x"]
        ewy = w.y - tgt["ang_vel"]["y"]
        ewz = w.z - tgt["ang_vel"]["z"]

        checks = {
            "pos.x": (abs(ex), self.pos_thr["x"]),
            "pos.y": (abs(ey), self.pos_thr["y"]),
            "pos.z": (abs(ez), self.pos_thr["z"]),
            "rpy.roll": (abs(eroll), self.rpy_thr["roll"]),
            "rpy.pitch": (abs(epitch), self.rpy_thr["pitch"]),
            "rpy.yaw": (abs(eyaw), self.rpy_thr["yaw"]),
            "vel.x": (abs(evx), self.vel_thr["x"]),
            "vel.y": (abs(evy), self.vel_thr["y"]),
            "vel.z": (abs(evz), self.vel_thr["z"]),
            "ang.x": (abs(ewx), self.ang_vel_thr["x"]),
            "ang.y": (abs(ewy), self.ang_vel_thr["y"]),
            "ang.z": (abs(ewz), self.ang_vel_thr["z"]),
        }

        failures = [
            f"{k}: {v:.4f} > {thr:.4f}"
            for k, (v, thr) in checks.items()
            if v > thr
        ]

        if failures:
            self.get_logger().warn("Waypoint blocked by: " + " | ".join(failures))

        ok = (
            abs(ex) <= self.pos_thr["x"]
            and abs(ey) <= self.pos_thr["y"]
            and abs(ez) <= self.pos_thr["z"]
            and abs(eroll) <= self.rpy_thr["roll"]
            and abs(epitch) <= self.rpy_thr["pitch"]
            and abs(eyaw) <= self.rpy_thr["yaw"]
            and abs(evx) <= self.vel_thr["x"]
            and abs(evy) <= self.vel_thr["y"]
            and abs(evz) <= self.vel_thr["z"]
            and abs(ewx) <= self.ang_vel_thr["x"]
            and abs(ewy) <= self.ang_vel_thr["y"]
            and abs(ewz) <= self.ang_vel_thr["z"]
        )
        self.get_logger().info(f"Reach ok: {ok}")

        if ok:
            if self._inside_since is None:
                self._inside_since = now
            elif (now - self._inside_since) >= self.dwell_s:
                if self.wp_idx < len(self.waypoints) - 1:
                    old = self.wp_idx
                    self.wp_idx += 1
                    self.get_logger().info(
                        f"Waypoint {old} satisfied thresholds for {self.dwell_s}s. "
                        f"Advancing -> {self.wp_idx}/{len(self.waypoints)-1}"
                    )
                    self._inside_since = None
                    self._cooldown_until = now + self.cooldown_s
        else:
            self._inside_since = None


def main(args=None):
    rclpy.init(args=args)
    node = WaypointTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()