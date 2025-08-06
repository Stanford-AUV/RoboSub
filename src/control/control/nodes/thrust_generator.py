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


class ThrustGenerator(Node):
    """This node converts a desired thrust vector into the individual thruster magnitudes and directions."""

    def __init__(self):
        """Initialize the ThrustGenerator node."""
        super().__init__("thrust_generator")
        # self.get_logger().info("SDKOFAJDSOFJAS")

        self.declare_parameter("timer_period", Parameter.Type.DOUBLE)
        self.declare_parameter("history_depth", Parameter.Type.INTEGER)
        self.declare_parameter("thruster_count", Parameter.Type.INTEGER)

        thruster_count = (
            self.get_parameter("thruster_count").get_parameter_value().integer_value
        )
        self._thruster_ids = [f"thruster_{i}" for i in range(thruster_count)]

        thruster_positions = np.empty(shape=(thruster_count, 3))
        thruster_orientations = np.empty(shape=(thruster_count, 3))
        for i, thruster_id in enumerate(self._thruster_ids):
            self.declare_parameter(
                f"thruster_positions.{thruster_id}", Parameter.Type.DOUBLE_ARRAY
            )
            self.declare_parameter(
                f"thruster_orientations.{thruster_id}", Parameter.Type.INTEGER_ARRAY
            )
            thruster_position = (
                self.get_parameter(f"thruster_positions.{thruster_id}")
                .get_parameter_value()
                .double_array_value
            )
            thruster_orientation = (
                self.get_parameter(f"thruster_orientations.{thruster_id}")
                .get_parameter_value()
                .integer_array_value
            )
            thruster_positions[i] = thruster_position
            thruster_orientations[i] = thruster_orientation / np.linalg.norm(
                thruster_orientation
            )
        self.TAM_inv = thruster_configs_to_TAM_inv(
            thruster_count, thruster_positions, thruster_orientations
        )

        # # --- 2) build your ENU‐map matrix from the axis_map ---
        # axis_map = {
        #     'x': ('z', -1),   # ENU X = –raw Z
        #     'y': ('x', -1),   # ENU Y = –raw X
        #     'z': ('y', +1),   # ENU Z =  raw Y
        # }
        # # R_hw[i,j] = sign means ENU-axis i takes raw-axis j times sign
        # self._R_hw = np.zeros((3,3))
        # for i, axis in enumerate(('x','y','z')):
        #     src, sgn = axis_map[axis]
        #     j = ('x','y','z').index(src)
        #     self._R_hw[i,j] = sgn

        self.wrench = Wrench()

        history_depth = (
            self.get_parameter("history_depth").get_parameter_value().integer_value
        )
        self._thrusts_pub = self.create_publisher(
            ThrustsStamped, "thrusts", history_depth
        )
        self._wrench_sub = self.create_subscription(
            WrenchStamped, "wrench", self.wrench_callback, history_depth
        )

        timer_period = (
            self.get_parameter("timer_period").get_parameter_value().double_value
        )
        # self.get_logger().info(f"dfsdfsd{self.get_parameter("timer_period").get_parameter_value().double_value}")
        self.get_logger().info(
            f"Converting wrench to individual thruster magnitudes every {timer_period} seconds"
        )
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        """Convert wrench to individual thruster magnitudes and publish them."""
        thrusts = total_force_to_individual_thrusts(self.TAM_inv, self.wrench)
        msg = ThrustsStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.thrusts = thrusts.tolist()
        # self.get_logger().info(f"Publishing wrench {self.wrench}")
        # self.get_logger().info(f"Publishing thrusts {msg.thrusts}")
        self._thrusts_pub.publish(msg)

    def wrench_callback(self, msg: WrenchStamped):
        """Handle incoming wrench messages."""
        self.wrench = msg.wrench
        self.get_logger().info(f"Received wrench {self.wrench}")

    # def wrench_callback(self, msg: WrenchStamped):
    #     """Receive the incoming wrench, remap axes into ENU, then store."""
    #     f = msg.wrench.force
    #     t = msg.wrench.torque

    #     # raw vectors in [x,y,z]
    #     f_raw = np.array([f.x, f.y, f.z], dtype=float)
    #     t_raw = np.array([t.x, t.y, t.z], dtype=float)

    #     # apply the same hw→ENU mapping to force & torque
    #     f_enu = self._R_hw.dot(f_raw)
    #     t_enu = self._R_hw.dot(t_raw)

    #     # assemble back into a Wrench
    #     w = msg.wrench
    #     w.force.x, w.force.y, w.force.z   = f_enu.tolist()
    #     w.torque.x, w.torque.y, w.torque.z = t_enu.tolist()

    #     self.wrench = w
    # self.get_logger().info(f"Received wrench {self.wrench}")
    # self.get_logger().info(f"Received wrench {f_raw} {f_enu}")


def main(args=None):
    """Initialize and spin the ThrustGenerator node."""
    rclpy.init(args=args)
    node = ThrustGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
