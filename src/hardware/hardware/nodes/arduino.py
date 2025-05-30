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

        self._sensors_pub = self.create_publisher(
            SensorsStamped, "sensors", history_depth
        )

        self._depth_pub = self.create_publisher(Float32Stamped, "depth", history_depth)

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

    def send_light(self):
        light = max(1100, min(self.light, 1900))  # 1100 to 1900
        command = f"light {light}"
        try:
            self.portName.write((command + "\n").encode())
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to write to serial port: {e}")
        self.portName.readline().decode().strip()

    def send_pwms(self):
        commands = [
            self.get_servo_command(index=i, pwm=pwm) for i, pwm in enumerate(self.pwms)
        ]
        message = " ".join(commands)
        try:
            self.portName.write((message + "\n").encode())
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to write to serial port: {e}")

    def update(self):
        if self.light_changed:
            self.send_light()
            self.light_changed = False
        self.send_pwms()
        response = self.portName.readline().decode().strip()
        parts = response.split(" ")
        if parts[0] != ">":
            self.get_logger().error(f"Unexpected response from Arduino: {response}")
            return
        sensors = {
            "pressure": None,
            "temperature": None,
            "depth": None,
            "current": None,
            "voltage": None,
        }
        for part in parts[1:]:
            name, value = part.split(":")
            if name == "servo":
                continue
            if name not in sensors:
                self.get_logger().error(f"Unknown sensor name: {name}")
            value = float(value)
            sensors[name] = value
        for name, value in sensors.items():
            if value is None:
                self.get_logger().error(f"Missing sensor value: {name}")
        msg = SensorsStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pressure = sensors["pressure"]
        msg.temperature = sensors["temperature"]
        msg.depth = sensors["depth"]
        msg.current = sensors["current"]
        msg.voltage = sensors["voltage"]
        # self.get_logger().info(f"Publishing sensors {msg}")
        self._sensors_pub.publish(msg)
        depth_msg = Float32Stamped()
        depth_msg.data = sensors["depth"]
        self._depth_pub.publish(depth_msg)

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
