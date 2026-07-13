#!/usr/bin/env python3
"""Live roll/pitch/yaw readout from /odometry/filtered (deg) for the sign test.

Rotate the sub CLOCKWISE by hand (viewed from above). In a correct FLU frame,
yaw should DECREASE. If yaw INCREASES, the orientation-estimate yaw sign is flipped.

Usage:
    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 yaw_check.py                 # reads /odometry/filtered
    python3 yaw_check.py /rotation       # reads raw IMU pose (pre-EKF)
"""
import sys
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from scipy.spatial.transform import Rotation as R


class YawCheck(Node):
    def __init__(self, topic):
        super().__init__("yaw_check")
        msg_type = PoseWithCovarianceStamped if topic == "/rotation" else Odometry
        self.create_subscription(msg_type, topic, self.cb, 10)
        self.get_logger().info(f"Reading {topic} — rotate CW, expect yaw to DECREASE")

    def cb(self, msg):
        o = msg.pose.pose.orientation
        roll, pitch, yaw = R.from_quat([o.x, o.y, o.z, o.w]).as_euler("xyz", degrees=True)
        wz = getattr(getattr(msg, "twist", None), "twist", None)
        wz = wz.angular.z if wz else float("nan")
        print(f"\rroll {roll:+7.1f}  pitch {pitch:+7.1f}  yaw {yaw:+7.1f}   gyro_z {wz:+6.2f}   ",
              end="", flush=True)


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else "/odometry/filtered"
    rclpy.init()
    node = YawCheck(topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
