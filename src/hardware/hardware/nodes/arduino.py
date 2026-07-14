import time

import serial
import rclpy
from rclpy.node import Node
from msgs.msg import PWMsStamped, SensorsStamped, Float32Stamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Int16, String
from typing import List
import numpy as np


class Arduino(Node):

    def __init__(self):
        super().__init__("arduino")

        self.zero_thrust = 1500
        self.light_changed = False
        self.light = 1100

        self.declare_parameter("history_depth", 10)
        self.declare_parameter("thruster_count", 8)

        self.thruster_count = (
            self.get_parameter("thruster_count").get_parameter_value().integer_value
        )
        self.pwms = [self.zero_thrust] * self.thruster_count
        # Failsafe: if the /pwms publisher (thrusters node) freezes or dies we
        # must not keep driving its last command forever (07/12 bag: node froze
        # 157s, sub kept spinning open-loop). No fresh /pwms within this window
        # -> latch neutral until messages resume. The Teensy heartbeat can't
        # catch this case: it keeps running while a ROS node is frozen.
        self.pwms_timeout = 0.5
        self.last_pwms_time = None
        self.pwms_stale = False

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
            SensorsStamped, "/arduino/sensors", history_depth
        )

        # Raw sensor-frame data from the two BNO085 IMUs, one topic per unit.
        # Consumed by hardware/nodes/bno085.py, which handles the remount into
        # base_link -- publish exactly what the firmware reports here.
        self._imu_pubs = [
            self.create_publisher(Imu, f"/arduino/imu_{i}", history_depth)
            for i in range(2)
        ]

        try:
            # udev symlink (99-teensy-acm.rules) pinned to the Teensy's USB
            # serial number. NEVER use a bare /dev/ttyACM<n>: re-enumeration
            # shuffles the numbers and 07/13 put a DaisySeed hydrophone board
            # on ttyACM0 -- PWMs would have streamed at the hydrophones.
            port = "/dev/ttyACM_teensy"
            self.portName = serial.Serial(port, baudrate=9600, timeout=1, exclusive=True)
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
        self.pwms: List[float] = msg.pwms.tolist()
        self.last_pwms_time = time.monotonic()
        if self.pwms_stale:
            self.pwms_stale = False
            self.get_logger().warning("/pwms resumed - leaving neutral failsafe")

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
        commands = []
        # for i, pwm in enumerate(self.pwms):
        #     if i == 0:
        #         pwm += (pwm - 1497) * 0.05
        #         pwm = int(pwm)
        #     commands.append(self.get_servo_command(index=i, pwm=pwm))
        commands = [
            self.get_servo_command(index=i, pwm=np.clip(pwm, 1250, 1750))
            for i, pwm in enumerate(self.pwms)
        ]
        message = " ".join(commands)
        try:
            self.portName.write((message + "\n").encode())
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to write to serial port: {e}")

    def update(self):
        if self.last_pwms_time is not None and not self.pwms_stale:
            if time.monotonic() - self.last_pwms_time > self.pwms_timeout:
                self.pwms_stale = True
                self.pwms = [self.zero_thrust] * self.thruster_count
                self.get_logger().error(
                    f"No /pwms for >{self.pwms_timeout}s - failsafe: driving neutral"
                )
        try:
            if self.light_changed:
                self.send_light()
                self.light_changed = False
            self.send_pwms()
            response = self.portName.readline().decode().strip().split("> ")[1]
            # self.get_logger().info(f"{response}")
            if response[0] == " ":
                response = response[1:]
            # self.get_logger().info(f"{response} is the response")
            tokens = response.split()
            data = {
                tokens[i].rstrip(":"): float(tokens[i + 1])
                for i in range(0, len(tokens), 2)
            }

            msg = SensorsStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "sensor"

            # self.get_logger().info(f"{data} is the data")
            if data == {}:
                self.get_logger().warning("No data received from Arduino.")
                return
            msg.pressure = data["pressure"]
            msg.depth = data["depth"]
            msg.external_temperature = data["external_temperature"]
            msg.internal_temperature1 = data["internal_temperature1"]
            msg.internal_temperature2 = data["internal_temperatur2"]
            msg.humidity = data["humidity"]
            msg.current = data["current"]
            msg.voltage = data["voltage"]

            self._sensors_pub.publish(msg)

            self.publish_imus(data)
        except:
            return

    def publish_imus(self, data):
        # TODO(firmware): the Arduino must append the BNO085 fields to the same
        # key/value response line, per unit n in {0, 1}:
        #   imu<n>_qw imu<n>_qx imu<n>_qy imu<n>_qz   (rotation vector quat)
        #   imu<n>_gx imu<n>_gy imu<n>_gz              (gyro, rad/s)
        #   imu<n>_ax imu<n>_ay imu<n>_az              (accel, m/s^2, gravity INCLUDED)
        # Until then this silently publishes nothing (keys missing).
        for i, pub in enumerate(self._imu_pubs):
            try:
                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = f"bno085_{i}"
                imu_msg.orientation.w = data[f"imu{i}_qw"]
                imu_msg.orientation.x = data[f"imu{i}_qx"]
                imu_msg.orientation.y = data[f"imu{i}_qy"]
                imu_msg.orientation.z = data[f"imu{i}_qz"]
                imu_msg.angular_velocity.x = data[f"imu{i}_gx"]
                imu_msg.angular_velocity.y = data[f"imu{i}_gy"]
                imu_msg.angular_velocity.z = data[f"imu{i}_gz"]
                imu_msg.linear_acceleration.x = data[f"imu{i}_ax"]
                imu_msg.linear_acceleration.y = data[f"imu{i}_ay"]
                imu_msg.linear_acceleration.z = data[f"imu{i}_az"]
            except KeyError:
                continue
            pub.publish(imu_msg)

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
