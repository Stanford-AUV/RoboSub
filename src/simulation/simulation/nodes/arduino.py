import rclpy
from rclpy.node import Node

from ros_gz_interfaces.msg import Altimeter
from msgs.msg import SensorsStamped

import numpy as np


class SimArduino(Node):
    """Stands in for hardware/arduino in sim: publishes /arduino/sensors, with depth from the Gazebo altimeter."""

    def __init__(self):
        super().__init__("sim_arduino")

        self.declare_parameter("surface_z", 0.0)
        self.declare_parameter("noise_spread", 0.03)
        self.surface_z = self.get_parameter("surface_z").value
        self.noise_spread = self.get_parameter("noise_spread").value

        self.sensor_publisher = self.create_publisher(SensorsStamped, "/arduino/sensors", 10)

        self.create_subscription(Altimeter, "gz/altimeter", self.on_altimeter, 10)

    def on_altimeter(self, msg: Altimeter):
        actual_height = msg.vertical_reference + msg.vertical_position
        depth = self.surface_z - actual_height
        noised_depth = depth + np.random.normal(0, self.noise_spread)

        # Only depth is simulated. Gazebo has no water temperature or battery, so the other
        # fields stay 0. Pressure units are undocumented: check a real bag before filling in.
        out = SensorsStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = "sensor"
        out.depth = float(noised_depth)
        self.sensor_publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    sim_arduino_node = SimArduino()
    rclpy.spin(sim_arduino_node)
    sim_arduino_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()