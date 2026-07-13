import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from msgs.msg import GeneratedPath  # Custom message import

import os
import yaml
import numpy as np
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory

from planning.utils.create_path import create_path


class PathGenerator(Node):
    def __init__(self):
        super().__init__("path_generator")

        # Waypoints are loaded IN-PROCESS from YAML at startup - there is no
        # /waypoints topic and no separate path_loader node. This deliberately
        # removes the DDS discovery race that used to drop the (one-shot,
        # latched) waypoints message when the loader published before this
        # node's subscription had matched, leaving the generator idle forever.
        # Override the file via the 'waypoints_path' ROS parameter if needed.
        # The YAML is installed into the package share dir (see setup.py), so
        # resolve it via ament - works under any install layout.
        default_yaml = os.path.join(
            get_package_share_directory("planning"), "prequal.yaml"
        )
        self.declare_parameter("waypoints_path", default_yaml)
        self.waypoints_path = (
            self.get_parameter("waypoints_path").get_parameter_value().string_value
            or default_yaml
        )
        # Allow a bare filename (e.g. "segments.yaml") - resolve it against the
        # package share dir so global.yaml doesn't need install-layout paths.
        if not os.path.isabs(self.waypoints_path) and not os.path.exists(
            self.waypoints_path
        ):
            self.waypoints_path = os.path.join(
                get_package_share_directory("planning"), self.waypoints_path
            )

        self.publish_desired = self.create_publisher(Odometry, "/desired/pose", 10)

        # The trajectory is a list of "legs". Each leg is a smooth spline path
        # ending at rest, optionally followed by a station-keeping pause on its
        # final pose (waypoint key `pause: <seconds>` in the YAML). Splitting at
        # pause waypoints - instead of splining through them - is what makes
        # the sub actually STOP there: a single spline through collinear points
        # carries velocity straight through them.
        self.legs = []  # list of (GeneratedPath, pause_after_sec)
        self.total_duration = 0.0
        self.path_start_time = None
        self.pause_logged = set()

        # Once the trajectory's duration has elapsed, keep holding the final
        # pose for this many seconds, then exit the node. The planning launch
        # marks this node with on_exit=Shutdown, and launch_sub.sh tears down
        # the whole stack (SIGINT + thruster neutralize) when any launch exits,
        # so path completion ends the run automatically. Set <0 to disable.
        self.declare_parameter("shutdown_after_path_sec", 3.0)
        self.shutdown_after_path_sec = (
            self.get_parameter("shutdown_after_path_sec")
            .get_parameter_value()
            .double_value
        )
        self.path_done_logged = False

        self.create_timer(1.0 / 60.0, self.publish_pose)

        self.get_logger().info("PathGenerator node has been started.")
        self.load_and_generate()

    def load_and_generate(self):
        """Read the waypoint YAML and build the trajectory. Runs once at startup."""
        self.get_logger().info(f"Loading waypoints from {self.waypoints_path}")

        try:
            with open(self.waypoints_path, "r") as f:
                data = yaml.safe_load(f)
        except Exception as exc:
            self.get_logger().error(
                f"Failed to read waypoints file '{self.waypoints_path}': {exc}"
            )
            return

        segments = [data[key] for key in data]

        waypoints = []  # list of (x, y, z, roll, pitch, yaw, pause_after)
        for segment in segments:
            for waypoint in segment["waypoints"]:
                waypoints.append(
                    (
                        waypoint["position"]["x"],
                        waypoint["position"]["y"],
                        waypoint["position"]["z"],
                        waypoint["orientation"]["roll"],
                        waypoint["orientation"]["pitch"],
                        waypoint["orientation"]["yaw"],
                        float(waypoint.get("pause", 0.0)),
                    )
                )

        if not waypoints:
            self.get_logger().error(
                f"No waypoints found in '{self.waypoints_path}'; nothing to generate."
            )
            return

        # Split the waypoint list into legs at every waypoint with pause > 0.
        # The pausing waypoint ends its leg AND seeds the next leg, so each leg
        # starts exactly where the previous one stopped.
        legs_waypoints = []
        current = []
        for wp in waypoints:
            current.append(wp)
            if wp[6] > 0.0:
                legs_waypoints.append((current, wp[6]))
                current = [wp]
        if len(current) > 1 or not legs_waypoints:
            legs_waypoints.append((current, 0.0))

        self.get_logger().info(
            f"Loaded {len(waypoints)} waypoints from {len(segments)} segment(s) "
            f"-> {len(legs_waypoints)} leg(s), generating path..."
        )

        legs = []
        total = 0.0
        for leg_wps, pause_after in legs_waypoints:
            arrays = [np.array([wp[i] for wp in leg_wps]) for i in range(6)]
            try:
                (
                    positions,
                    velocities,
                    accelerations,
                    orientations,
                    angular_velocities,
                    angular_accelerations,
                    duration,
                ) = create_path(*arrays)
            except Exception as exc:
                self.get_logger().error(
                    f"Failed to generate path from waypoints: {exc}"
                )
                return
            leg_path = self.make_generated_path(
                positions, velocities, orientations, angular_velocities, duration
            )
            legs.append((leg_path, pause_after))
            total += duration + pause_after

        self.legs = legs
        self.total_duration = total
        self.path_start_time = self.get_clock().now()
        self.get_logger().info(
            f"Generated {len(legs)} leg(s), total duration {total:.1f}s "
            "(including pauses)."
        )

    def make_generated_path(
        self, positions, velocities, orientations, angular_velocities, duration
    ):

        generated_path = GeneratedPath()
        generated_path.header.stamp = self.get_clock().now().to_msg()
        generated_path.header.frame_id = "map"
        generated_path.duration = duration

        for i in range(len(positions[0])):
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = generated_path.header.stamp
            pose_stamped.header.frame_id = generated_path.header.frame_id

            pose_stamped.pose.position.x = positions[0][i]
            pose_stamped.pose.position.y = positions[1][i]
            pose_stamped.pose.position.z = positions[2][i]

            orientation_quat = Rotation.from_euler(
                "xyz", orientations[i], degrees=True
            ).as_quat()
            pose_stamped.pose.orientation.x = orientation_quat[0]
            pose_stamped.pose.orientation.y = orientation_quat[1]
            pose_stamped.pose.orientation.z = orientation_quat[2]
            pose_stamped.pose.orientation.w = orientation_quat[3]

            generated_path.poses.append(pose_stamped)

            twist = Twist()
            twist.linear.x = velocities[0][i]
            twist.linear.y = velocities[1][i]
            twist.linear.z = velocities[2][i]
            twist.angular.x = angular_velocities[i][0]
            twist.angular.y = angular_velocities[i][1]
            twist.angular.z = angular_velocities[i][2]

            generated_path.twists.append(twist)

        return generated_path

    def publish_pose(self):
        if not self.legs or self.path_start_time is None:
            # No trajectory yet - make the idle state loud instead of silent so a
            # failed load is obvious in the terminal (throttled to avoid spam).
            self.get_logger().warn(
                "No generated path yet; not publishing /desired/pose",
                throttle_duration_sec=5.0,
            )
            return

        elapsed = (self.get_clock().now() - self.path_start_time).nanoseconds / 1e9

        # Walk the leg/pause timeline to find where `elapsed` lands.
        t = max(elapsed, 0.0)
        for i, (leg, pause_after) in enumerate(self.legs):
            n = len(leg.poses)
            if n == 0:
                continue
            if t < leg.duration:
                index = min(int(t / leg.duration * (n - 1)), n - 1)
                odom = Odometry()
                odom.header = leg.poses[index].header
                odom.pose.pose = leg.poses[index].pose
                odom.twist.twist = leg.twists[index]
                self.publish_desired.publish(odom)
                return
            t -= max(leg.duration, 0.0)
            if t < pause_after:
                # Pause phase: station-keep on the leg's final pose with ZERO
                # velocity feed-forward (same reasoning as the end-of-path
                # hold: re-sending twists here makes the sub thrash).
                if i not in self.pause_logged:
                    self.pause_logged.add(i)
                    self.get_logger().info(
                        f"Leg {i + 1}/{len(self.legs)} done; pausing "
                        f"{pause_after:.1f}s at its final pose."
                    )
                odom = Odometry()
                odom.header = leg.poses[-1].header
                odom.pose.pose = leg.poses[-1].pose
                odom.twist.twist = Twist()
                self.publish_desired.publish(odom)
                return
            t -= pause_after

        # All legs and pauses elapsed: hold the very last pose, then exit.
        if not self.path_done_logged:
            self.path_done_logged = True
            self.get_logger().info(
                f"Path complete ({self.total_duration:.1f}s); holding final pose "
                f"{self.shutdown_after_path_sec:.1f}s before shutting down."
            )
        if (
            self.shutdown_after_path_sec >= 0.0
            and elapsed >= self.total_duration + self.shutdown_after_path_sec
        ):
            self.get_logger().info("Hold elapsed - exiting to shut down the stack.")
            raise SystemExit

        last_leg = self.legs[-1][0]
        if len(last_leg.poses) == 0:
            return
        odom = Odometry()
        odom.header = last_leg.poses[-1].header
        odom.pose.pose = last_leg.poses[-1].pose
        # Hold phase: station-keep on the final pose with ZERO velocity
        # feed-forward. Re-sending the last twist here kept commanding
        # motion after the path ended and made the sub thrash pre-shutdown.
        odom.twist.twist = Twist()
        self.publish_desired.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    try:
        rclpy.spin(node)
    except SystemExit:
        # Raised from publish_pose once the path (+ hold) is done: exit 0 so
        # ros2 launch treats it as a clean shutdown, not a crash.
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
