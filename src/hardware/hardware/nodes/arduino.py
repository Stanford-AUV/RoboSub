import serial
import rclpy
from rclpy.node import Node
from msgs.msg import PWMsStamped, SensorsStamped, Float32Stamped
from std_msgs.msg import Int16
from typing import List
from rclpy import Parameter
import numpy as np


class Arduino(Node):

    def __init__(self):
        super().__init__("arduino")

        self.zero_thrust = 1500
        self.light_changed = False
        self.light = 1100

        self.declare_parameter("history_depth", Parameter.Type.INTEGER)
        self.declare_parameter("thruster_count", Parameter.Type.INTEGER)

        self.thruster_count = (
            self.get_parameter("thruster_count").get_parameter_value().integer_value
        )
        self.pwms = [self.zero_thrust] * self.thruster_count

        history_depth = (
            self.get_parameter("history_depth").get_parameter_value().integer_value
        )
        self._pwms_sub = self.create_subscription(
            PWMsStamped, "pwms", self.pwms_callback, history_depth
        )

        self._light_sub = self.create_subscription(
            Int16, "light", self.light_callback, history_depth
        )

        self._torpedo_sub = self.create_subscription(
            Int16, "torpedo", self.torpedo_callback, history_depth
        )

        self._torpedo_sub = self.create_subscription(
            Int16, "dropper", self.dropper_callback, history_depth
        )

        self._sensors_pub = self.create_publisher(
            SensorsStamped, "/arduino/sensors", history_depth
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

        self.timer = self.create_timer(0.01, self.update)

        self.left_torpedo = False
        self.right_torpedo = False

        self.left_dropper = False
        self.right_dropper = False

    def get_servo_command(self, index: int, pwm: int):
        if pwm == 0:
            pwm = self.zero_thrust
        command = f"{pwm}"
        return command

    def pwms_callback(self, msg: PWMsStamped):
        # self.get_logger().info(f"PWMs received {msg.pwms}")
        self.pwms: List[float] = msg.pwms.tolist()

    def light_callback(self, msg: Int16):
        light = msg.data
        if light != self.light:
            self.light_changed = True
            self.light = light

    def torpedo_callback(self, msg: Int16):
        # light = msg.data
        # if light != self.light:
        #     self.light_changed = True
        #     self.light = light
        if msg.data >= 0.5 and not self.left_torpedo:
            command = "nagasaki"
            self.left_torpedo = True
        elif msg.data <= -0.5 and not self.right_torpedo:
            command = "hiroshima"
            self.right_torpedo = True
        else:
            command = None

        if command is not None:
            self.get_logger().info("boom")
            try:
                self.portName.write((command + "\n").encode())
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to write to serial port: {e}")
            self.portName.readline().decode().strip()

    def dropper_callback(self, msg: Int16):
        # light = msg.data
        # if light != self.light:
        #     self.light_changed = True
        #     self.light = light
        if msg.data >= 0.5:# and not self.left_dropper:
            command = "pearl-harbor"
            self.left_dropper = True
        elif msg.data <= -0.5:# and not self.right_dropper:
            command = "iwo-jima"
            self.right_dropper = True
        else:
            command = "dropperPosition 0"

        if command is not None:
            if msg.data >= 0.5 or msg.data <= -0.5:
                self.get_logger().info("boom dropper")
            try:
                self.portName.write((command + "\n").encode())
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to write to serial port: {e}")
            self.portName.readline().decode().strip()

    def send_light(self):
        light = max(1100, min(self.light, 1900))  # 1100 to 1900
        command = f"light {light}"
        try:
            self.portName.write((command + "\n").encode())
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to write to serial port: {e}")
        self.portName.readline().decode().strip()

    def send_pwms(self):
        # self.get_logger().info("I am born!!!")
        commands = [
            self.get_servo_command(index=i, pwm=pwm) for i, pwm in enumerate(self.pwms)
        ]
        # self.get_logger().info("I am servoed!!!")
        message = " ".join(commands)
        try:
            # self.get_logger().info(f":DdDDDDDDD YAY! {message}")
            # message = "1500 " * 8
            # message = message[:-1]
            temp = (message + "\n").encode()
            # self.get_logger().info("Gonna write bruh")
            self.portName.write(temp)
            # self.get_logger().info("Got wrote bruh")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to write to serial port: {e}")

    def update(self):
        # self.get_logger().error(f"Enter update loop!")
        if self.light_changed:
            self.send_light()
            self.light_changed = False
        # self.get_logger().error(f"Checkpoint A!")
        self.send_pwms()
        # self.get_logger().error(f"Checkpoint B!")
        response = self.portName.readline().decode().strip()
        tokens = response.split()
        data = {tokens[i].rstrip(':'): float(tokens[i+1]) for i in range(0, len(tokens), 2)}

        msg = SensorsStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "sensor"

        msg.pressure = data["pressure"]
        msg.depth = data["depth"]
        msg.external_temperature = data["external_temperature"]
        msg.internal_temperature1 = data["internal_temperature1"]
        msg.internal_temperature2 = data["internal_temperature2"]
        msg.humidity = data["humidity"]
        msg.current = data["current"]
        msg.voltage = data["voltage"]

        self._sensors_pub.publish(msg)

    def kill_motors(self):
        self.pwms = [self.zero_thrust] * self.thruster_count
        self.send_pwms()


def main(args=None):
    rclpy.init(args=args)
    arduino = Arduino()

    try:
        rclpy.spin(arduino)
    except Exception as e:
        arduino.kill_motors()
        raise e

    arduino.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
