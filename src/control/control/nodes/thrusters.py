import numpy as np
import rclpy
from geometry_msgs.msg import Wrench, WrenchStamped
from rclpy import Parameter
from rclpy.node import Node
from std_msgs.msg import Float64

from control.utils.thrusters import (
    thruster_configs_to_TAM_inv,
    total_force_to_individual_thrusts,
)


class Thrusters(Node):
    """This node converts a desired thrust vector into the individual thruster magnitudes and directions."""

    def __init__(self):
        super().__init__("thrusters")

        self.declare_parameter("timer_period", Parameter.Type.DOUBLE)
        self.declare_parameter("history_depth", Parameter.Type.INTEGER)
        self.declare_parameter("thruster_count", Parameter.Type.INTEGER)

        thruster_count = (
            self.get_parameter("thruster_count").get_parameter_value().integer_value
        )
        self._thruster_ids = [f"thruster_{i + 1}" for i in range(thruster_count)]

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

        self.wrench = Wrench()

        self._thrust_pubs = []
        history_depth = (
            self.get_parameter("history_depth").get_parameter_value().integer_value
        )
        for thruster_id in self._thruster_ids:
            thrust_pub = self.create_publisher(
                Float64, f"thrusters/{thruster_id}/magnitude", history_depth
            )
            self._thrust_pubs.append(thrust_pub)
        self._wrench_sub = self.create_subscription(
            Wrench, "wrench", self.wrench_callback, history_depth
        )

        timer_period = (
            self.get_parameter("timer_period").get_parameter_value().double_value
        )
        self.get_logger().info(
            f"Converting wrench to individual thruster magnitudes every {timer_period} seconds"
        )
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        thrusts = total_force_to_individual_thrusts(self.TAM_inv, self.wrench)
        for i, thrust in enumerate(thrusts.tolist()):
            msg = Float64()
            msg.data = thrust
            self._thrust_pubs[i].publish(msg)

    def wrench_callback(self, msg: WrenchStamped):
        self.wrench = msg.wrench
        self.get_logger().info(f"Received wrench {self.wrench}")


def main(args=None):
    rclpy.init(args=args)
    node = Thrusters()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
