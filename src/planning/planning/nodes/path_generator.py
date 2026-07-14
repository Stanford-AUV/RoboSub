import os

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory

from planning.utils.bake import load_bake, source_hash
from planning.utils.geometry import override_yaw
from planning.utils.pursuit import arrived, pursuit_target, step_toward

PURSUIT_V_MAX = 0.25  # m/s - matches create_path's max_velocity
PURSUIT_GOAL_STALE_SEC = 2.0
PURSUIT_ARRIVE_HOLD_SEC = 1.0
TICK_HZ = 60.0


class PathGenerator(Node):
    """Plays a baked trajectory (tools/bake_path.py): spline legs sampled on
    shore, plus go_to_object pursuit segments servoing on the live world goal
    from perception. No spline math happens at runtime."""

    def __init__(self):
        super().__init__("path_generator")

        default_yaml = os.path.join(
            get_package_share_directory("planning"), "prequal.yaml"
        )
        self.declare_parameter("waypoints_path", default_yaml)
        self.waypoints_path = (
            self.get_parameter("waypoints_path").get_parameter_value().string_value
            or default_yaml
        )
        if not os.path.isabs(self.waypoints_path) and not os.path.exists(
            self.waypoints_path
        ):
            self.waypoints_path = os.path.join(
                get_package_share_directory("planning"), self.waypoints_path
            )

        # Once the trajectory is done, keep holding the final pose for this
        # many seconds, then exit. The planning launch marks this node with
        # on_exit=Shutdown, and launch_sub.sh tears down the whole stack when
        # any launch exits, so path completion ends the run automatically.
        # Set <0 to disable.
        self.declare_parameter("shutdown_after_path_sec", 3.0)
        self.shutdown_after_path_sec = (
            self.get_parameter("shutdown_after_path_sec")
            .get_parameter_value()
            .double_value
        )

        self.publish_desired = self.create_publisher(Odometry, "/desired/pose", 10)

        # Live yaw for yaw:free samples: publishing the measured yaw as the
        # desired yaw gives the PID zero yaw error, so it applies no yaw
        # torque while the mask is active (D-term still damps spin).
        self.meas_yaw = None
        self.create_subscription(
            Odometry, "/odometry/filtered", self.on_odom, 10
        )

        self.items = self.load_baked()
        self.item_index = 0
        self.item_start = None  # rclpy Time, set on first tick of each item
        self.pause_logged = set()
        self.path_done_logged = False
        self.done_time = None

        # Pursuit state
        self.cmd_pos = None  # np.ndarray(3,) - last commanded position
        self.cmd_yaw = 0.0  # deg
        self.pursuit_entry_pos = None
        self.pursuit_target_pos = None
        self.pursuit_target_yaw = None
        self.pursuit_arrived_at = None
        self.pursuit_fallback_logged = False
        self.goals = {}  # object_id -> (np.ndarray(3,), rclpy Time)
        for oid in {
            i["object_id"] for i in self.items if i["type"] == "go_to_object"
        }:
            self.create_subscription(
                PointStamped,
                f"/object/{oid}/world_position",
                lambda msg, oid=oid: self.on_goal(oid, msg),
                10,
            )

        self.create_timer(1.0 / TICK_HZ, self.tick)
        self.get_logger().info(
            f"Playing baked path: {len(self.items)} item(s) "
            f"({sum(1 for i in self.items if i['type'] == 'leg')} leg(s))"
        )

    def load_baked(self):
        base, _ = os.path.splitext(self.waypoints_path)
        baked_path = base + ".baked.yaml"
        try:
            doc = load_bake(baked_path)
        except Exception as exc:
            self.get_logger().error(
                f"Cannot read baked path '{baked_path}': {exc}. "
                "Run: python tools/bake_path.py <waypoints yaml> on shore."
            )
            raise SystemExit(1)
        if doc.get("source_sha256") != source_hash(self.waypoints_path):
            self.get_logger().error(
                f"Baked path '{baked_path}' is STALE for "
                f"'{self.waypoints_path}'. Re-run tools/bake_path.py."
            )
            raise SystemExit(1)
        return doc["items"]

    def on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self.meas_yaw = float(
            Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler(
                "xyz", degrees=True
            )[2]
        )

    def on_goal(self, object_id, msg: PointStamped):
        p = np.array([msg.point.x, msg.point.y, msg.point.z])
        if np.all(np.isfinite(p)):
            self.goals[object_id] = (p, self.get_clock().now())

    def fresh_goal(self, object_id):
        entry = self.goals.get(object_id)
        if entry is None:
            return None
        p, t = entry
        if (self.get_clock().now() - t).nanoseconds / 1e9 > PURSUIT_GOAL_STALE_SEC:
            return None
        return p

    def publish_cmd(self, pos, quat_xyzw, twist=None):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "map"
        odom.pose.pose.position.x = float(pos[0])
        odom.pose.pose.position.y = float(pos[1])
        odom.pose.pose.position.z = float(pos[2])
        odom.pose.pose.orientation.x = float(quat_xyzw[0])
        odom.pose.pose.orientation.y = float(quat_xyzw[1])
        odom.pose.pose.orientation.z = float(quat_xyzw[2])
        odom.pose.pose.orientation.w = float(quat_xyzw[3])
        odom.twist.twist = twist if twist is not None else Twist()
        self.publish_desired.publish(odom)
        self.cmd_pos = np.asarray(pos, dtype=float)
        self.cmd_yaw = float(
            Rotation.from_quat(quat_xyzw).as_euler("xyz", degrees=True)[2]
        )

    def leg_pose(self, item, i):
        """Position + quat for leg sample i; releases yaw to the measured
        yaw when the baked yaw_free mask marks the sample. Returns
        (pos, quat_xyzw, yaw_free)."""
        pose = item["poses"][i]
        mask = item.get("yaw_free")
        if not (mask and mask[i]):
            return pose[:3], pose[3:], False
        yaw = self.meas_yaw if self.meas_yaw is not None else self.cmd_yaw
        return pose[:3], override_yaw(pose[3:], yaw), True

    def advance_item(self):
        self.item_index += 1
        self.item_start = None
        self.pursuit_entry_pos = None
        self.pursuit_target_pos = None
        self.pursuit_target_yaw = None
        self.pursuit_arrived_at = None
        self.pursuit_fallback_logged = False

    def tick(self):
        if self.item_index >= len(self.items):
            self.finish_tick()
            return
        item = self.items[self.item_index]
        if self.item_start is None:
            self.item_start = self.get_clock().now()
        if item["type"] == "leg":
            self.leg_tick(item)
        else:
            self.pursuit_tick(item)

    def elapsed(self):
        return (self.get_clock().now() - self.item_start).nanoseconds / 1e9

    def leg_tick(self, item):
        t = self.elapsed()
        poses, twists = item["poses"], item["twists"]
        n = len(poses)
        duration = max(item["duration"], 1e-9)
        if t < item["duration"]:
            i = min(int(t / duration * (n - 1)), n - 1)
            pos, quat, yaw_free = self.leg_pose(item, i)
            tw = Twist()
            (
                tw.linear.x,
                tw.linear.y,
                tw.linear.z,
                tw.angular.x,
                tw.angular.y,
                tw.angular.z,
            ) = [float(v) for v in twists[i]]
            if yaw_free:
                # No angular feed-forward while yaw is released.
                tw.angular.x = tw.angular.y = tw.angular.z = 0.0
            self.publish_cmd(pos, quat, tw)
            return
        if t < item["duration"] + item["pause_after"]:
            # Station-keep on the leg's final pose with ZERO velocity
            # feed-forward (re-sending twists makes the sub thrash).
            if self.item_index not in self.pause_logged:
                self.pause_logged.add(self.item_index)
                self.get_logger().info(
                    f"Item {self.item_index + 1}/{len(self.items)} done; "
                    f"pausing {item['pause_after']:.1f}s."
                )
            pos, quat, _ = self.leg_pose(item, n - 1)
            self.publish_cmd(pos, quat)
            return
        pos, quat, _ = self.leg_pose(item, n - 1)
        self.publish_cmd(pos, quat)
        self.advance_item()

    def pursuit_tick(self, item):
        now = self.get_clock().now()
        dt = 1.0 / TICK_HZ
        if self.cmd_pos is None:
            # Pursuit as the first item: no commanded pose yet - hold origin
            # until a goal shows up (control holds current pose anyway).
            self.cmd_pos = np.zeros(3)
        if self.pursuit_entry_pos is None:
            self.pursuit_entry_pos = self.cmd_pos.copy()
            self.get_logger().info(
                f"go_to_object '{item['object_id']}' "
                f"(standoff={item['standoff']}m timeout={item['timeout']}s)"
            )

        goal = self.fresh_goal(item["object_id"])
        if goal is not None:
            # Approach line anchored at the ENTRY position so the standoff
            # point doesn't slide as the sub moves.
            target, yaw = pursuit_target(
                goal, self.pursuit_entry_pos, item["standoff"]
            )
            self.pursuit_target_pos, self.pursuit_target_yaw = target, yaw
        elif (
            self.pursuit_target_pos is None
            and self.elapsed() > item["timeout"]
        ):
            # Never saw it: give up and drive to the fallback waypoint.
            if not self.pursuit_fallback_logged:
                self.pursuit_fallback_logged = True
                self.get_logger().warn(
                    f"'{item['object_id']}' not seen in {item['timeout']:.0f}s; "
                    "driving to fallback waypoint."
                )
            fb = item["fallback"]
            self.pursuit_target_pos = np.array(fb[:3])
            self.pursuit_target_yaw = fb[5]

        if self.pursuit_target_pos is None:
            # Waiting for first detection: hold the entry pose.
            quat = Rotation.from_euler(
                "xyz", [0.0, 0.0, self.cmd_yaw], degrees=True
            ).as_quat()
            self.publish_cmd(self.pursuit_entry_pos, quat)
            return

        new_cmd = step_toward(
            self.cmd_pos, self.pursuit_target_pos, PURSUIT_V_MAX, dt
        )
        quat = Rotation.from_euler(
            "xyz", [0.0, 0.0, self.pursuit_target_yaw], degrees=True
        ).as_quat()
        self.publish_cmd(new_cmd, quat)

        if arrived(new_cmd, self.pursuit_target_pos, item["arrive_tol"]):
            if self.pursuit_arrived_at is None:
                self.pursuit_arrived_at = now
                self.get_logger().info(
                    f"Arrived at '{item['object_id']}' target; holding "
                    f"{PURSUIT_ARRIVE_HOLD_SEC:.0f}s."
                )
            elif (
                now - self.pursuit_arrived_at
            ).nanoseconds / 1e9 >= PURSUIT_ARRIVE_HOLD_SEC:
                self.advance_item()
        else:
            self.pursuit_arrived_at = None

    def finish_tick(self):
        # All items done: hold the final commanded pose (zero twist - velocity
        # feed-forward here made the sub thrash pre-shutdown), then exit.
        if not self.path_done_logged:
            self.path_done_logged = True
            self.done_time = self.get_clock().now()
            self.get_logger().info(
                f"Path complete; holding {self.shutdown_after_path_sec:.1f}s "
                "before shutting down."
            )
        if self.cmd_pos is not None:
            quat = Rotation.from_euler(
                "xyz", [0.0, 0.0, self.cmd_yaw], degrees=True
            ).as_quat()
            self.publish_cmd(self.cmd_pos, quat)
        if (
            self.shutdown_after_path_sec >= 0.0
            and (self.get_clock().now() - self.done_time).nanoseconds / 1e9
            >= self.shutdown_after_path_sec
        ):
            self.get_logger().info("Hold elapsed - exiting to shut down the stack.")
            raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PathGenerator()
    except SystemExit:
        # Startup failure (missing/stale bake): propagate a nonzero exit so
        # the launch treats it as a crash and aborts on shore.
        rclpy.shutdown()
        raise
    try:
        rclpy.spin(node)
    except SystemExit:
        # Raised from finish_tick once the path (+ hold) is done: exit 0 so
        # ros2 launch treats it as a clean shutdown, not a crash.
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
