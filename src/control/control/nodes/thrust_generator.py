"""This module contains the ThrustGenerator node which converts a desired thrust vector into the individual thruster magnitudes and directions."""

import numpy as np
import rclpy
from geometry_msgs.msg import Wrench, WrenchStamped
from rclpy import Parameter
from rclpy.node import Node
from msgs.msg import ThrustsStamped
import yaml
import os

from control.utils.thrust_generator import (
    thruster_configs_to_TAM_inv,
    total_force_to_individual_thrusts,
)


class ThrustGenerator(Node):
    """This node converts a desired thrust vector into the individual thruster magnitudes and directions."""

    def __init__(self, path):
        """Initialize the ThrustGenerator node."""
        super().__init__("thrust_generator")

        self.path = path

        thruster_positions, thruster_orientations = self.get_yaml_info()
        self.TAM_inv = thruster_configs_to_TAM_inv(
            thruster_positions, thruster_orientations
        )

        self.wrench = Wrench()

        self._thrusts_pub = self.create_publisher(ThrustsStamped, "/thrusts", 10)
        self._wrench_sub = self.create_subscription(
            WrenchStamped, "/wrench", self.wrench_callback, 10
        )

        timer_period = 1.0 / 60.0
        self.get_logger().info(
            f"Converting wrench to individual thruster magnitudes every {timer_period} seconds"
        )
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def get_yaml_info(self):
        if not os.path.exists(self.path):
            raise FileNotFoundError("Noooooo! No yaml path exists :(")

        with open(self.path, "r") as f:
            data = yaml.safe_load(f)

        thruster_positions = []
        thruster_orientations = []
        for key in data:
            val = data.get(key)
            pos = [val["x"], val["y"], val["z"]]
            ori = [val["dx"], val["dy"], val["dz"]]
            thruster_positions.append(pos)
            thruster_orientations.append(ori)
        return np.array(thruster_positions, dtype=float), np.array(
            thruster_orientations, dtype=float
        )

    def timer_callback(self):
        """Convert wrench to individual thruster magnitudes and publish them."""
        thrusts = total_force_to_individual_thrusts(self.TAM_inv, self.wrench)
        msg = ThrustsStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.thrusts = thrusts.tolist()

        self._thrusts_pub.publish(msg)

    def wrench_callback(self, msg: WrenchStamped):
        """Handle incoming wrench messages."""
        self.wrench = msg.wrench


def main(args=None):
    """Initialize and spin the ThrustGenerator node."""
    rclpy.init(args=args)

    SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
    yaml_path = os.path.join(
        SCRIPT_DIR, "..", "..", "..", "hardware", "hardware", "thrusters.yaml"
    )
    yaml_path = os.path.abspath(yaml_path)

    node = ThrustGenerator(yaml_path)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
