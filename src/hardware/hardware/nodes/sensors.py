"""This node converts sensor messages sent from the hardware into a unified format for use for state estimation."""

import rclpy
from rclpy import Parameter
import math
from rclpy.node import Node
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from message_filters import Subscriber, ApproximateTimeSynchronizer
from msgs.msg import DVLData, DVLBeam, DVLTarget, DVLVelocity, Float32Stamped
import numpy as np
import random

class Sensors(Node):

    def __init__(self):
        super().__init__("sensors")

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

        self.get_logger().info(f"Listening to DVL and IMU")

    def sync_callback_imu_only(self, imu_msg: Imu):
        imu_msg.header.frame_id = "imu_frame"
        imu_msg.header.stamp = self.get_clock().now().to_msg()

        imu_msg.angular_velocity_covariance = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]
        imu_msg.linear_acceleration_covariance = [
            0.2, 0.0, 0.0,
            0.0, 0.2, 0.0,
            0.0, 0.0, 0.2
        ]

        # Always publish imu0 (gyro+accel)
        self.sync_imu_publisher_.publish(imu_msg)

        # Build pose message for pose1 (orientation)
        imu_pose_msg = PoseWithCovarianceStamped()
        imu_pose_msg.header.stamp = imu_msg.header.stamp
        imu_pose_msg.header.frame_id = "imu_frame"

        # Pose covariance: ONLY orientation entries nonzero
        imu_pose_msg.pose.covariance = [
            1.0,  0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  1.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  1.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  1,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  1,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  1
        ]

        # Validate + normalize quaternion
        q = np.array([
            imu_msg.orientation.x,
            imu_msg.orientation.y,
            imu_msg.orientation.z,
            imu_msg.orientation.w
        ], dtype=float)

        n = np.linalg.norm(q)
        if (not np.isfinite(q).all()) or n < 1e-6:
            self.get_logger().warn("Invalid IMU quaternion; NOT publishing /imu/pose_sync this cycle")
            return

        q /= n
        imu_pose_msg.pose.pose.orientation.x = float(q[0])
        imu_pose_msg.pose.pose.orientation.y = float(q[1])
        imu_pose_msg.pose.pose.orientation.z = float(q[2])
        imu_pose_msg.pose.pose.orientation.w = float(q[3])

        # Publish pose1 ONLY if valid
        self.sync_imu_pose_publisher_.publish(imu_pose_msg)


    def sync_callback_dvl_only(self, dvl_msg: DVLData):
        dvl_twist_msg = TwistWithCovarianceStamped()
        dvl_twist_msg.twist.twist.linear = dvl_msg.velocity.mean
        dvl_twist_msg.header.stamp = dvl_msg.header.stamp
        dvl_twist_msg.header.frame_id = "dvl_frame"
        # fmt: off
        dvl_twist_msg.twist.covariance = [
            0.1, 0.0, 0.0,  0.0, 0.0, 0.0,
            0.0, 0.1, 0.0,  0.0, 0.0, 0.0,
            0.0, 0.0, 0.1,  0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,  0.1, 0.0, 0.0,
            0.0, 0.0, 0.0,  0.0, 0.1, 0.0,
            0.0, 0.0, 0.0,  0.0, 0.0, 0.1
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
