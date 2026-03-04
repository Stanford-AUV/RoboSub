##### NOTE: This node is not used in the current implementation (ekf.yaml). It is kept here for reference."

"""This node converts sensor messages sent from the hardware into a unified format for use for state estimation."""

import rclpy
from rclpy import Parameter
from rclpy.node import Node
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseWithCovarianceStamped
from msgs.msg import DVLData


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
        self.sync_depth_publisher_ = self.create_publisher(
            PoseWithCovarianceStamped, "/depth/pose_sync", history_depth
        )

        self.dvl_only_sub = self.create_subscription(
            DVLData, "dvl", self.sync_callback_dvl_only, history_depth
        )

        self.get_logger().info("Listening to DVL")

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
