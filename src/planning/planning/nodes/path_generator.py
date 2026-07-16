import os

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory

from planning.utils.bake import load_bake, source_hash
from planning.utils.geometry import override_yaw
from planning.utils.pursuit import arrived, pursuit_target, step_toward

TICK_HZ = 60.0
# Decision samples to collect before committing to a branch option; we take
# the mode so a few noisy perception frames can't flip the choice.
# Branch decisions: station-keep and listen to the decision topic this long,
# then vote on the value CHANGES seen (see branch_decision below).
BRANCH_LISTEN_SEC = 10.0


def branch_decision(codes):
    """Decide a branch from the sample buffer collected while listening.

    The decision topic (/pinger) republishes its latched value at 5 Hz, so
    raw samples are dominated by stickiness. What carries information is
    each SWITCH: the first sample and every subsequent change count one
    vote for the value switched to; the majority wins; a tie goes to the
    most recent value. Returns (code, votes).
    """
    votes = {}
    prev = None
    for c in codes:
        if c != prev:
            votes[c] = votes.get(c, 0) + 1
            prev = c
    best = max(votes.values())
    winners = [c for c, n in votes.items() if n == best]
    return (winners[0] if len(winners) == 1 else codes[-1]), votes


class PathGenerator(Node):
    """Plays a baked trajectory (tools/bake_path.py): spline legs sampled on
    shore. No spline math happens at runtime."""

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
        # Branch playback: while sitting on a branch item we subscribe to its
        # decision topic and buffer the reported codes here until we have
        # enough to commit. The chosen option's legs are then spliced into
        # self.items, so leg_tick plays them like any other leg.
        self.branch_subscriber = None
        self.branch_code = []

        self.items = self.load_baked()
        # Actuator triggers: a leg can carry a `publish` topic (baked from a
        # waypoint's `publish:` key). Pre-create a String publisher for every
        # such topic - including those inside branch options, whose legs are
        # spliced in at runtime - so DDS discovery is already complete when a
        # leg finishes and fires. Create-publish-then-destroy would drop the
        # message: the actuator node hasn't matched the publisher yet.
        self.trigger_publishers = {
            topic: self.create_publisher(String, topic, 10)
            for topic in self._publish_topics(self.items)
        }
        # Branch listen-window signals: "<decision>/listening" carries
        # "start" when the branch begins listening and "stop" when it
        # commits, so the daisy node can save a debug plot of exactly that
        # window. Pre-created for the same DDS-discovery reason as above.
        self.listen_publishers = {
            item["decision"]: self.create_publisher(
                String, item["decision"] + "/listening", 10)
            for item in self.items if item["type"] == "branch"
        }
        self.item_index = 0
        self.item_start = None  # rclpy Time, set on first tick of each item
        self.pause_logged = set()
        self.path_done_logged = False
        self.done_time = None

        # Pursuit state
        self.cmd_pos = None  # np.ndarray(3,) - last commanded position
        self.cmd_yaw = 0.0  # deg

        # Mission stage for dashboards/plots: published on every change
        # plus a 1 Hz keepalive so late-starting subscribers catch up.
        self.stage_pub = self.create_publisher(String, "/mission/stage", 10)
        self._stage = ""
        self.create_timer(1.0, lambda: self._stage and
                          self.stage_pub.publish(String(data=self._stage)))

        self.create_timer(1.0 / TICK_HZ, self.tick)
        self.get_logger().info(
            f"Playing baked path: {len(self.items)} item(s) "
            f"({sum(1 for i in self.items if i['type'] == 'leg')} leg(s))"
        )

    def set_stage(self, stage):
        if stage != self._stage:
            self._stage = stage
            self.stage_pub.publish(String(data=stage))
            self.get_logger().info(f"Stage: {stage}")

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

    @staticmethod
    def _publish_topics(items):
        """All `publish` topics reachable from these items, recursing into
        branch options (their legs get spliced in at runtime)."""
        topics = set()
        for item in items:
            if item["type"] == "leg" and item.get("publish"):
                topics.add(item["publish"])
            elif item["type"] == "branch":
                for opt in item["options"]:
                    topics |= PathGenerator._publish_topics(opt["items"])
        return topics

    def on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self.meas_yaw = float(
            Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler(
                "xyz", degrees=True
            )[2]
        )

    def on_branch(self, msg: String):
        self.branch_code.append(msg.data)

    def hold_pose(self):
        """Station-keep on the last commanded pose with zero velocity."""
        if self.cmd_pos is None:
            return
        quat = Rotation.from_euler(
            "xyz", [0.0, 0.0, self.cmd_yaw], degrees=True
        ).as_quat()
        self.publish_cmd(self.cmd_pos, quat)

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
        # Tear down any branch decision subscription and clear its buffer so
        # the next branch starts fresh (no-op for leg items).
        if self.branch_subscriber is not None:
            self.destroy_subscription(self.branch_subscriber)
            self.branch_subscriber = None
        self.branch_code = []


    def tick(self):
        if self.item_index >= len(self.items):
            self.set_stage("mission done")
            self.finish_tick()
            return
        item = self.items[self.item_index]
        if self.item_start is None:
            self.item_start = self.get_clock().now()
        if item["type"] == "leg":
            self.set_stage(f"leg {self.item_index + 1}/{len(self.items)}")
            self.leg_tick(item)
        elif item["type"] == "branch":
            if self.branch_subscriber is None:
                self.branch_subscriber = self.create_subscription(
                    String, item["decision"], self.on_branch, 10
                )
                self.get_logger().info(
                    f"Branch: holding, listening on {item['decision']} "
                    f"for {BRANCH_LISTEN_SEC:.0f} s."
                )
                self.listen_publishers[item["decision"]].publish(
                    String(data="start"))
            self.set_stage(f"branch {item['decision']}: listening")
            self.branch_tick(item)

    def elapsed(self):
        return (self.get_clock().now() - self.item_start).nanoseconds / 1e9

    def select_option(self, item, code):
        """Pick the option whose code matches the decision; codes are compared
        as strings so an Int-in-YAML and a String message still match."""
        for opt in item["options"]:
            if str(opt["code"]) == str(code):
                return opt
        self.get_logger().warn(
            f"Branch {item['decision']}: code {code!r} matched no option; "
            "using the first."
        )
        return item["options"][0]

    def branch_tick(self, item):
        # Station-keep on the pose we entered the branch with for the full
        # listen window (and until at least one sample arrived), then vote
        # on the switches seen.
        self.hold_pose()
        if not self.branch_code or self.elapsed() < BRANCH_LISTEN_SEC:
            return
        code, votes = branch_decision(self.branch_code)
        self.listen_publishers[item["decision"]].publish(String(data="stop"))
        opt = self.select_option(item, code)
        self.get_logger().info(
            f"Branch {item['decision']}: switch votes {votes} -> "
            f"code {code!r}, playing option with {len(opt['items'])} leg(s)."
        )
        # Splice the chosen option's legs in right after this branch item, then
        # advance onto the first of them - leg_tick handles them from here.
        self.items[self.item_index + 1 : self.item_index + 1] = opt["items"]
        self.advance_item()

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
        topic = item.get("publish")
        if topic:
            self.trigger_publishers[topic].publish(String(data="pub"))
            self.get_logger().info(f"Leg done; fired trigger on {topic}.")
        self.advance_item()

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
