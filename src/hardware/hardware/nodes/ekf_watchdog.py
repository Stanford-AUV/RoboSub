"""EKF divergence watchdog.

robot_localization keeps attitude as euler RPY, so a trip near pitch +-90
(flip during handling, aggressive maneuver) can corrupt the filter, and the
pose1 rejection threshold can latch: once the EKF's yaw is far from the
IMU's, every /rotation message is >N sigma away and gets rejected forever,
leaving yaw free-wheeling ("spins out of control").

This node watches /odometry/filtered against the IMU's /rotation and resets
the filter via /set_pose when it has clearly diverged:
  - any non-finite value in the odometry pose/twist, or
  - orientation disagreement > DISAGREE_RAD sustained > DISAGREE_SEC.
The reset keeps the current position estimate (nothing better exists) and
snaps orientation back to the IMU's, which is the only absolute attitude
reference on the vehicle.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

DISAGREE_RAD = math.radians(45)
DISAGREE_SEC = 3.0
RESET_COOLDOWN_SEC = 10.0


def _quat_angle(q1, q2):
    """Angle (rad) between two orientations given as (x, y, z, w)."""
    dot = abs(sum(a * b for a, b in zip(q1, q2)))
    return 2.0 * math.acos(min(1.0, dot))


class EkfWatchdog(Node):
    def __init__(self):
        super().__init__("ekf_watchdog")
        self._imu_quat = None
        self._disagree_since = None
        self._last_reset = -1e9

        self.create_subscription(
            PoseWithCovarianceStamped, "/rotation", self._rotation_cb, 10
        )
        self.create_subscription(
            Odometry, "/odometry/filtered", self._odom_cb, 10
        )
        self._set_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/set_pose", 1
        )
        self.get_logger().info("EKF watchdog running")

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _rotation_cb(self, msg):
        q = msg.pose.pose.orientation
        self._imu_quat = (q.x, q.y, q.z, q.w)

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t = msg.twist.twist
        values = [
            p.x, p.y, p.z, q.x, q.y, q.z, q.w,
            t.linear.x, t.linear.y, t.linear.z,
            t.angular.x, t.angular.y, t.angular.z,
        ]
        if not all(np.isfinite(values)):
            self._reset(msg, reason="non-finite odometry state")
            return

        if self._imu_quat is None:
            return
        angle = _quat_angle(self._imu_quat, (q.x, q.y, q.z, q.w))
        if angle < DISAGREE_RAD:
            self._disagree_since = None
            return
        now = self._now()
        if self._disagree_since is None:
            self._disagree_since = now
        elif now - self._disagree_since > DISAGREE_SEC:
            self._reset(
                msg,
                reason=(
                    f"EKF vs IMU orientation off by "
                    f"{math.degrees(angle):.0f} deg for >{DISAGREE_SEC:.0f}s"
                ),
            )

    def _reset(self, odom, reason):
        now = self._now()
        if now - self._last_reset < RESET_COOLDOWN_SEC or self._imu_quat is None:
            return
        self._last_reset = now
        self._disagree_since = None

        out = PoseWithCovarianceStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "odom"
        p = odom.pose.pose.position
        pos = [p.x, p.y, p.z]
        if not all(np.isfinite(pos)):
            pos = [0.0, 0.0, 0.0]
        out.pose.pose.position.x = pos[0]
        out.pose.pose.position.y = pos[1]
        out.pose.pose.position.z = pos[2]
        qx, qy, qz, qw = self._imu_quat
        out.pose.pose.orientation.x = qx
        out.pose.pose.orientation.y = qy
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw
        cov = np.zeros(36)
        for i in range(3):
            cov[i * 7] = 0.25       # keep position, moderate confidence
            cov[(i + 3) * 7] = 0.05  # orientation from IMU
        out.pose.covariance = cov.tolist()
        self._set_pose_pub.publish(out)
        self.get_logger().warn(f"EKF reset via /set_pose: {reason}")


def main(args=None):
    rclpy.init(args=args)
    node = EkfWatchdog()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
