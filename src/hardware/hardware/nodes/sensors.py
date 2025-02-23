"""This node converts sensor messages sent from the hardware into a unified format for use for state estimation."""

import rclpy
from rclpy import Parameter
import math
from rclpy.node import Node
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from message_filters import Subscriber, ApproximateTimeSynchronizer
from std_msgs.msg import Float32
from msgs.msg import DVLData, DVLBeam, DVLTarget, DVLVelocity

class Sensors(Node):

    def __init__(self):
        super().__init__("sensors")

        self.declare_parameter("history_depth", Parameter.Type.INTEGER)
        self.declare_parameter("slop", Parameter.Type.DOUBLE)

        history_depth = (
            self.get_parameter("history_depth").get_parameter_value().integer_value
        )
        slop = self.get_parameter("slop").get_parameter_value().double_value

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
        self.last_imu_sync_ts_sec = math.nan

        self.dvl_sub = Subscriber(self, DVLData, "dvl") # Double-check
        self.imu_sub = Subscriber(self, Imu, "imu")
        self.depth_sub = Subscriber(self, Float32, "depth") # Double-check
        self.imu_only_sub = self.create_subscription(
            Imu, "imu", self.sync_callback_D, history_depth
        )

        # synchronize DVL, IMU, Depth data
        self.ts_caseA = ApproximateTimeSynchronizer(  # case A: all messages available
            [self.dvl_sub, self.imu_sub, self.depth_sub],
            queue_size=history_depth,
            slop=slop,
        )
        self.ts_caseA.registerCallback(self.sync_callback_A)

        self.ts_caseB = ApproximateTimeSynchronizer(  # case B: IMU DVL available
            [self.dvl_sub, self.imu_sub], queue_size=history_depth, slop=slop
        )
        self.ts_caseB.registerCallback(self.sync_callback_B)

        self.ts_caseC = ApproximateTimeSynchronizer(  # case C: IMU Depth available
            [self.imu_sub, self.depth_sub], queue_size=history_depth, slop=slop
        )
        self.ts_caseC.registerCallback(self.sync_callback_C)

        self.get_logger().info(f"Running IMU/DVL/Depth data synchronization")

    def sync_callback_A(self, dvl_msg, imu_msg, depth_msg):
        # self.get_logger().info(f"A")
        self.last_imu_sync_ts_sec = (
            imu_msg.header.stamp
        )  # update most recent IMU time for valid full sync
        imu_msg.header.stamp = self.last_imu_sync_ts_sec

        imu_pose_msg = PoseWithCovarianceStamped()
        imu_pose_msg.header.stamp = self.last_imu_sync_ts_sec
        imu_pose_msg.header.frame_id = "odom"
        imu_pose_msg.pose.pose.orientation.x = imu_msg.orientation.x
        imu_pose_msg.pose.pose.orientation.y = imu_msg.orientation.y
        imu_pose_msg.pose.pose.orientation.z = imu_msg.orientation.z
        imu_pose_msg.pose.pose.orientation.w = imu_msg.orientation.w

        dvl_twist_msg = TwistWithCovarianceStamped()
        dvl_twist_msg.twist.twist.linear = dvl_msg.velocity.mean
        dvl_twist_msg.header.stamp = self.last_imu_sync_ts_sec
        self.sync_dvl_publisher_.publish(dvl_twist_msg)
        
        self.sync_imu_publisher_.publish(imu_msg)
        self.sync_imu_pose_publisher_.publish(imu_pose_msg)

        depth_pose_msg = PoseWithCovarianceStamped()
        depth_pose_msg.pose.pose.position.z = depth_msg.data
        depth_pose_msg.header.stamp = self.last_imu_sync_ts_sec
        self.sync_depth_publisher_.publish(depth_pose_msg)

    def sync_callback_B(self, dvl_msg, imu_msg):
        # self.get_logger().info(f"B")
        CurrTS = imu_msg.header.stamp
        if (
            CurrTS != self.last_imu_sync_ts_sec
        ):  # if time is the same as full sync, ignore since we've already pub'd
            imu_msg.header.stamp = CurrTS

            imu_pose_msg = PoseWithCovarianceStamped()
            imu_pose_msg.header.stamp = CurrTS
            imu_pose_msg.header.frame_id = "odom"
            imu_pose_msg.pose.pose.orientation.x = imu_msg.orientation.x
            imu_pose_msg.pose.pose.orientation.y = imu_msg.orientation.y
            imu_pose_msg.pose.pose.orientation.z = imu_msg.orientation.z
            imu_pose_msg.pose.pose.orientation.w = imu_msg.orientation.w

            dvl_twist_msg = TwistWithCovarianceStamped()
            dvl_twist_msg.twist.twist.linear = dvl_msg.velocity.mean
            dvl_twist_msg.header.stamp = CurrTS
            self.sync_dvl_publisher_.publish(dvl_twist_msg)

            self.sync_imu_publisher_.publish(imu_msg)
            self.sync_imu_pose_publisher_.publish(imu_pose_msg)

    def sync_callback_C(self, imu_msg, depth_msg):
        # self.get_logger().info(f"C")
        CurrTS = imu_msg.header.stamp
        if (
            CurrTS != self.last_imu_sync_ts_sec
        ):  # if time is the same as full sync, ignore since we've already pub'd
            imu_msg.header.stamp = CurrTS

            imu_pose_msg = PoseWithCovarianceStamped()
            imu_pose_msg.header.stamp = CurrTS
            imu_pose_msg.header.frame_id = "odom"
            imu_pose_msg.pose.pose.orientation.x = imu_msg.orientation.x
            imu_pose_msg.pose.pose.orientation.y = imu_msg.orientation.y
            imu_pose_msg.pose.pose.orientation.z = imu_msg.orientation.z
            imu_pose_msg.pose.pose.orientation.w = imu_msg.orientation.w

            self.sync_imu_publisher_.publish(imu_msg)
            self.sync_imu_pose_publisher_.publish(imu_pose_msg)

            depth_pose_msg = PoseWithCovarianceStamped()
            depth_pose_msg.pose.pose.position.z = depth_msg.data
            depth_pose_msg.header.stamp = CurrTS
            self.sync_depth_publisher_.publish(depth_pose_msg)

    def sync_callback_D(self, imu_msg):
        self.get_logger().info(f"D")
        CurrTS = imu_msg.header.stamp
        if (
            CurrTS != self.last_imu_sync_ts_sec
        ):  # if time is the same as full sync, ignore since we've already pub'd
            imu_msg.header.stamp = CurrTS

            imu_pose_msg = PoseWithCovarianceStamped()
            imu_pose_msg.header.stamp = CurrTS
            imu_pose_msg.header.frame_id = "odom"
            imu_pose_msg.pose.pose.orientation.x = imu_msg.orientation.x
            imu_pose_msg.pose.pose.orientation.y = imu_msg.orientation.y
            imu_pose_msg.pose.pose.orientation.z = imu_msg.orientation.z
            imu_pose_msg.pose.pose.orientation.w = imu_msg.orientation.w

            self.sync_imu_publisher_.publish(imu_msg)
            self.sync_imu_pose_publisher_.publish(imu_pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Sensors()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
