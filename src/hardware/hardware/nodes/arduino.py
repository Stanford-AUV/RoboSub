import serial
import rclpy
from rclpy.node import Node
from msgs.msg import PWMsStamped
from typing import List
from rclpy import Parameter
import numpy as np


class Arduino(Node):

    def __init__(self):
        super().__init__("arduino")

        self.zero_thrust = 1500

        self.declare_parameter("history_depth", Parameter.Type.INTEGER)
        self.declare_parameter("thruster_count", Parameter.Type.INTEGER)

        self.thruster_count = (
            self.get_parameter("thruster_count").get_parameter_value().integer_value
        )

        history_depth = (
            self.get_parameter("history_depth").get_parameter_value().integer_value
        )
        self._pwms_sub = self.create_subscription(
            PWMsStamped, "pwms", self.pwms_callback, history_depth
        )

        try:
            # NOTE: If this fails, run the following command:
            # sudo chmod a+rw /dev/ttyACM0
            port = "/dev/ttyACM0"
            self.portName = serial.Serial(port, baudrate=9600, timeout=1)
            self.get_logger().info(f"Serial port {port} opened successfully.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise e

        # TODO: Request temperature and humidity in a timer

    def get_servo_command(self, index: int, pwm: int):
        servo_number = index
        if pwm == 0:
            pwm = self.zero_thrust
        command = f"{servo_number} {pwm}\n"
        return command

    def pwms_callback(self, msg: PWMsStamped, log=True):
        if log:
            self.get_logger().info(f"PWMs received {msg.pwms}")
        pwms: List[int] = msg.pwms.tolist()
        commands = [
            self.get_servo_command(index=i, pwm=pwm) for i, pwm in enumerate(pwms)
        ]
        try:
            self.portName.write("".join(commands).encode())
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to write to serial port: {e}")
        response = self.portName.readlines()
        expected_messages = set(
            [f"Set servo {i} to {pwm}" for i, pwm in enumerate(pwms)]
        )
        for i, response in enumerate(response):
            response = response.decode().strip()
            if response in expected_messages:
                expected_messages.remove(response)
            else:
                self.get_logger().error(f"Thruster index {i}: {response}")

    def kill_motors(self):
        msgs = PWMsStamped()
        msgs.pwms = np.array([0] * self.thruster_count, dtype=np.int16)
        self.pwms_callback(msgs, log=False)


def main(args=None):
    rclpy.init(args=args)
    arduino = Arduino()

    try:
        rclpy.spin(arduino)
    except KeyboardInterrupt:
        arduino.kill_motors()
        return

    arduino.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
