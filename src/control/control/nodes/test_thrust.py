"""This module contains the ThrustGenerator node which converts a desired thrust vector into the individual thruster magnitudes and directions."""

import numpy as np
import rclpy
from geometry_msgs.msg import Wrench, WrenchStamped
from rclpy import Parameter
from rclpy.node import Node
from msgs.msg import ThrustsStamped

from control.utils.thrust_generator import (
    thruster_configs_to_TAM_inv,
    total_force_to_individual_thrusts,
)


class TestThrust(Node):
    """This node converts a desired thrust vector into the individual thruster magnitudes and directions."""

    def __init__(self):
        """Initialize the ThrustGenerator node."""
        super().__init__("thrust_generator")

        self._thrusts_pub = self.create_publisher(
            ThrustsStamped, "thrusts", 10
        )
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        """Convert wrench to individual thruster magnitudes and publish them."""
        msg = ThrustsStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.thrusts = [0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._thrusts_pub.publish(msg)


def main(args=None):
    """Initialize and spin the TestThrust node."""
    rclpy.init(args=args)
    node = TestThrust()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
