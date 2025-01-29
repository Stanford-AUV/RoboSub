import serial
import rclpy
from rclpy.node import Node
from msgs.msg import PWMsStamped
from typing import List


class Arduino(Node):

    def __init__(self):
        super().__init__("arduino")

        self.zero_thrust = 1500

        self.thrusters_count = 8
        self._pwms_sub = self.create_subscription(
            PWMsStamped, "pwms", self.pwms_callback, 10
        )

        try:
            self.portName = serial.Serial(
                "/dev/ttyUSB_teensy", baudrate=9600, timeout=1
            )
            self.get_logger().info(
                f"Serial port /dev/ttyUSB_teensy opened successfully."
            )
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            return

        # TODO: Request temperature and humidity in a timer

    def get_servo_command(self, index: int, pwm: int):
        servo_number = index + 2
        command = f"{servo_number} {pwm}\n"
        self.get_logger().info(command)
        return command

    def pwms_callback(self, msg: PWMsStamped):
        pwms: List[int] = msg.pwms.tolist()
        for i, pwm in enumerate(pwms):
            command = self.get_servo_command(index=i, pwm=pwm)
            try:
                self.portName.write(command.encode())
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to write to serial port: {e}")

    def kill_motors(self):
        self.get_logger().info("Kill command received")
        result = True
        for i in range(self.thrusters_count):
            command = self.get_servo_command(index=i, pwm=self.zero_thrust)
            try:
                self.portName.write(command.encode())
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to write to serial port: {e}")
                result = False
        return result


def main(args=None):
    rclpy.init(args=args)
    arduino = Arduino()

    try:
        rclpy.spin(arduino)
    except KeyboardInterrupt:
        pass

    arduino.kill_motors()
    arduino.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
