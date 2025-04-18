import numpy as np
import rclpy
from rclpy import Parameter
from rclpy.node import Node
from msgs.msg import ThrustsStamped, PWMsStamped
from msgs.msg import SensorsStamped

from hardware.utils.thrusters import thrust_to_pwm

TIME_AVERAGING_WINDOW = 10


class Thrusters(Node):
    """This node converts individual thruster magnitudes and directions into PWM signals to control the thrusters."""

    def __init__(self):
        super().__init__("thrusters")

        self.declare_parameter("timer_period", Parameter.Type.DOUBLE)
        self.declare_parameter("history_depth", Parameter.Type.INTEGER)
        self.declare_parameter("thruster_count", Parameter.Type.INTEGER)

        self.last_voltages = [14.8]

        thruster_count = (
            self.get_parameter("thruster_count").get_parameter_value().integer_value
        )

        self.pwms = np.zeros(thruster_count, dtype=np.int16)

        history_depth = (
            self.get_parameter("history_depth").get_parameter_value().integer_value
        )
        self._thrusts_sub = self.create_subscription(
            ThrustsStamped, "thrusts", self.thrusts_callback, history_depth
        )
        self._pwms_pub = self.create_publisher(PWMsStamped, "pwms", history_depth)
        self._sensors_sub = self.create_subscription(
            SensorsStamped, "sensors", self.sensors_callback, history_depth
        )

        timer_period = (
            self.get_parameter("timer_period").get_parameter_value().double_value
        )
        self.get_logger().info(
            f"Converting thrusts to motor PWMs every {timer_period} seconds"
        )
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def get_voltage(self):
        # Time-averaged voltage
        assert len(self.last_voltages) <= TIME_AVERAGING_WINDOW
        return np.mean(self.last_voltages)

    def thrusts_callback(self, msg: ThrustsStamped):
        try:
            voltage = self.get_voltage()
            self.get_logger().info(f"Voltage {voltage}")
            self.get_logger().info(f"Received thrusts {msg.thrusts}")
            self.pwms = np.array(
                [thrust_to_pwm(thrust, voltage) for thrust in msg.thrusts],
                dtype=np.int16,
            )
        except ValueError as e:
            self.get_logger().error(f"Failed to convert thrusts to PWMs: {e}")

    def sensors_callback(self, msg: SensorsStamped):
        self.last_voltages.append(msg.voltage)
        if len(self.last_voltages) > TIME_AVERAGING_WINDOW:
            self.last_voltages.pop(0)

    def timer_callback(self):
        msg = PWMsStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pwms = self.pwms
        self.get_logger().info(f"Publishing PWMs {msg.pwms}")
        self._pwms_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Thrusters()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
