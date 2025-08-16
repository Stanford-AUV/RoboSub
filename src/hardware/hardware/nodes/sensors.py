"""This node converts sensor messages sent from the hardware into a unified format for use for state estimation."""

import rclpy
from rclpy import Parameter
import math
from rclpy.node import Node
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from message_filters import Subscriber, ApproximateTimeSynchronizer
from msgs.msg import DVLData, DVLBeam, DVLTarget, DVLVelocity, Float32Stamped
import random
from msgs.msg import Float32Stamped

from collections import deque
import math
from rclpy.time import Time


class Sensors(Node):

    def __init__(self):
        super().__init__("sensors")
        self.get_logger().info("Starting [sensors]")

        self.declare_parameter("history_depth", Parameter.Type.INTEGER)

        history_depth = (
            self.get_parameter("history_depth").get_parameter_value().integer_value
        )
        self.sync_dvl_publisher_ = self.create_publisher(
            TwistWithCovarianceStamped, "/dvl/twist_sync", history_depth
        )
        self.sync_imu_publisher_ = self.create_publisher(
            Imu, "/imu/data_sync", history_depth
        )
        self.sync_imu_pose_publisher_ = self.create_publisher(
            PoseWithCovarianceStamped, "/imu/pose_sync", history_depth
        )
        self.sync_depth_publisher_ = self.create_publisher(
            PoseWithCovarianceStamped, "/depth/pose_sync", history_depth
        )

        self.imu_only_sub = self.create_subscription(
            Imu, "imu", self.sync_callback_imu_only, history_depth
        )
        self.dvl_only_sub = self.create_subscription(
            DVLData, "dvl", self.sync_callback_dvl_only, history_depth
        )

        self.depth_subscription = self.create_subscription(
            Float32Stamped, 'depth', self.depth_callback, 10
        )

        # ---- filter config (tune as needed) ----
        self.depth_win_size = 21               # odd number is nice for median
        self.depth_sigma_k = 3.5               # outlier threshold in MAD units
        self.depth_ema_alpha = 0.25            # 0..1 (higher = less smoothing)
        self.depth_max_rate = 0.5              # meters/sec allowed change
        self.depth_min = -50.0                 # optional clamp
        self.depth_max =  50.0

        # ---- filter state ----
        self._depth_buf = deque(maxlen=self.depth_win_size)
        self._depth_last_ema = None
        self._depth_last_time = None

        self.get_logger().info(f"Listening to DVL and IMU")

    def _mad(self, vals, median):
        # median absolute deviation (robust scale)
        abs_dev = [abs(v - median) for v in vals]
        n = len(abs_dev)
        if n == 0:
            return 0.0
        abs_dev.sort()
        return abs_dev[n // 2]

    def _filter_depth(self, z_raw: float, t_msg: Time) -> float:
        """
        Returns a filtered depth using:
        - rolling median + MAD outlier rejection
        - EMA smoothing
        - rate limiting on dz/dt
        """

        # 1) push to window, compute rolling median
        self._depth_buf.append(z_raw)
        vals = list(self._depth_buf)
        vals.sort()
        med = vals[len(vals) // 2]
        mad = self._mad(vals, med) or 1e-6  # avoid divide-by-zero

        # 2) reject outliers vs median (replace with median)
        if abs(z_raw - med) > self.depth_sigma_k * mad:
            z_used = med
        else:
            z_used = z_raw

        # 3) EMA smoothing
        if self._depth_last_ema is None:
            z_ema = z_used
        else:
            a = self.depth_ema_alpha
            z_ema = a * z_used + (1.0 - a) * self._depth_last_ema

        # 4) rate limiting (dz/dt)
        now = t_msg
        if self._depth_last_time is None:
            dt = 0.0
        else:
            dt_ns = (now - self._depth_last_time).nanoseconds
            dt = max(dt_ns / 1e9, 0.0)

        if dt > 0 and self._depth_last_ema is not None:
            max_step = self.depth_max_rate * dt
            dz = z_ema - self._depth_last_ema
            if dz > max_step:
                z_ema = self._depth_last_ema + max_step
            elif dz < -max_step:
                z_ema = self._depth_last_ema - max_step

        # 5) clamp to sane bounds
        z_ema = max(self.depth_min, min(self.depth_max, z_ema))

        # update state
        self._depth_last_ema = z_ema
        self._depth_last_time = now
        return z_ema


    def depth_callback(self, depth_msg: Float32Stamped):
        depth_pose_msg = PoseWithCovarianceStamped()
        if depth_msg.header.stamp.sec == 0 and depth_msg.header.stamp.nanosec == 0:
            depth_pose_msg.header.stamp = self.get_clock().now().to_msg()
            t = self.get_clock().now()
        else:
            depth_pose_msg.header.stamp = depth_msg.header.stamp
            t = Time.from_msg(depth_msg.header.stamp)

        z_filtered = self._filter_depth(float(depth_msg.data), t)

        depth_pose_msg.header.frame_id = "odom"
        depth_pose_msg.pose.pose.position.z = z_filtered
        # self.get_logger().info(f"depth raw={depth_msg.data:.3f} -> filt={z_filtered:.3f}")

        depth_pose_msg.pose.covariance = [
            0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0025, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0
        ]

        self.sync_depth_publisher_.publish(depth_pose_msg)

    def sync_callback_imu_only(self, imu_msg: Imu):
        imu_msg.header.frame_id = "odom"
        imu_msg.header.stamp = self.get_clock().now().to_msg() #imu_msg.header.stamp

        # imu_msg.orientation_covariance = [
        #     0.05, 0.0, 0.0,   # roll  ±√0.05 rad (~13°)
        #     0.0, 0.05, 0.0,   # pitch ±√0.05 rad (~13°)
        #     0.0, 0.0, 0.05    # yaw   ±√0.05 rad (~13°)
        # ]

        # fmt: off
        imu_msg.angular_velocity_covariance = [
            0.05, 0.0, 0.0,
            0.0, 0.05, 0.0,
            0.0, 0.0, 0.05
        ]
        # fmt: off
        imu_msg.linear_acceleration_covariance = [
            0.05, 0.0, 0.0,
            0.0, 0.05, 0.0,
            0.0, 0.0, 0.05
        ]

        imu_msg.orientation_covariance = [
            0.05, 0.0, 0.0,
            0.0, 0.05, 0.0,
            0.0, 0.0, 0.05
        ]

        imu_pose_msg = PoseWithCovarianceStamped()
        imu_pose_msg.header.stamp = self.get_clock().now().to_msg() #imu_msg.header.stamp
        imu_pose_msg.header.frame_id = "odom"
        imu_pose_msg.pose.pose.orientation.x = imu_msg.orientation.x
        imu_pose_msg.pose.pose.orientation.y = imu_msg.orientation.y
        imu_pose_msg.pose.pose.orientation.z = imu_msg.orientation.z
        imu_pose_msg.pose.pose.orientation.w = imu_msg.orientation.w
        # fmt: off
        imu_pose_msg.pose.covariance = [
            0.25,  0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.25,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.25,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.25,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.25,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.25
        ]

        self.sync_imu_publisher_.publish(imu_msg)
        self.sync_imu_pose_publisher_.publish(imu_pose_msg)

    def sync_callback_dvl_only(self, dvl_msg: DVLData):
        dvl_twist_msg = TwistWithCovarianceStamped()
        dvl_twist_msg.twist.twist.linear = dvl_msg.velocity.mean
        dvl_twist_msg.header.stamp = dvl_msg.header.stamp
        dvl_twist_msg.header.frame_id = "base_link"
        # fmt: off
        dvl_twist_msg.twist.covariance = [
            0.05, 0.0, 0.0,  0.0, 0.0, 0.0,
            0.0, 0.05, 0.0,  0.0, 0.0, 0.0,
            0.0, 0.0, 0.05,  0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,  0.05, 0.0, 0.0,
            0.0, 0.0, 0.0,  0.0, 0.05, 0.0,
            0.0, 0.0, 0.0,  0.0, 0.0, 0.05
        ]
        self.sync_dvl_publisher_.publish(dvl_twist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Sensors()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
