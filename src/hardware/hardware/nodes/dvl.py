import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped

from hardware.utils.dvl_utils.system import OutputData
from hardware.utils.dvl_utils.dvl_connect import Dvl
from hardware.nodes.generic_sensor import GenericSensor
import numpy as np
import glob
import serial

DEBUG_MODE = False

_BIG = 1e9


class DVL(GenericSensor):
    def __init__(self, baudrate=115200):
        super().__init__("dvl_ros_bridge", "dvl_0")

        self.sensor_publishers = {
            "position": self.create_publisher(PoseWithCovarianceStamped, "/position", 10),
            "velocity": self.create_publisher(TwistWithCovarianceStamped, "/velocity", 10),
        }

        self._latest_data = None

        if DEBUG_MODE:
            self.get_logger().info("Running in DEBUG_MODE, publishing dummy data.")
            self.timer = self.create_timer(0.1, self._publish_dummy)
            return

        self.dvl = self.autodetect_dvl_port(baudrate)
        if not self.dvl:
            self.get_logger().error("Failed to connect to DVL.")
            return

        self.dvl.register_ondata_callback(self.update_data)

        if not self.dvl.exit_command_mode():
            self.get_logger().error("Failed to start pinging")

    def autodetect_dvl_port(self, baudrate, timeout=2):
        """Scan available serial ports and attempt to connect to the DVL."""
        possible_ports = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
        for port in possible_ports:
            try:
                with serial.Serial(port, baudrate, timeout=timeout) as ser:
                    if not ser.is_open:
                        continue

                dvl = Dvl(port, baudrate)
                self.get_logger().info(f"Port: {port}")
                if dvl.is_connected():
                    return dvl
                self.get_logger().error(f"Failed to connect to DVL on port {port}")

            except serial.SerialException:
                self.get_logger().debug(f"Port {port} is not available or is busy.")
            except serial.SerialTimeoutException:
                self.get_logger().debug(f"Port {port} timed out.")
            except Exception as e:
                self.get_logger().error(f"Unexpected error on port {port}: {e}")
        return None

    def update_data(self, output_data: OutputData, obj):
        del obj
        if output_data is None:
            return

        settings = output_data.get_settings()
        data_dict = {setting.name: setting.value for setting in settings}

        self._latest_data = data_dict
        self.publish_sensor_data()

    
    def _build_twist_cov(self, velocity_error=None):
        cov = np.zeros(36)
        vel_axes = self.get_axes("velocity")
        vel_cov = self.get_covariance("velocity")

        for i in range(3):
            cov[i * 6 + i] = _BIG

        if vel_axes:
            if velocity_error is not None:
                var = velocity_error ** 2
                for axis in vel_axes:
                    idx = axis - 1
                    cov[idx * 6 + idx] = var
            elif vel_cov is not None:
                for r in range(3):
                    for c in range(3):
                        if (r + 1) in vel_axes and (c + 1) in vel_axes:
                            cov[r * 6 + c] = float(vel_cov[r][c])

        cov[3 * 6 + 3] = _BIG
        cov[4 * 6 + 4] = _BIG
        cov[5 * 6 + 5] = _BIG

        return cov.tolist()

    def _build_pose_cov(self):
        cov = np.zeros(36)
        pos_axes = self.get_axes("position")
        pos_cov = self.get_covariance("position")

        for i in range(3):
            cov[i * 6 + i] = _BIG

        if pos_cov is not None and pos_axes:
            for axis in pos_axes:
                idx = axis - 1
                cov[idx * 6 + idx] = float(pos_cov[idx][idx])

        return cov.tolist()

    def publish_sensor_data(self):
        d = self._latest_data
        if d is None:
            return

        stamp = self.get_clock().now().to_msg()

        if self.is_active("velocity"):
            vx = self._safe(d["Velocity X"])
            vy = self._safe(d["Velocity Y"])
            vz = self._safe(d["Velocity Z"])
            vel = self.R_sensor_to_base @ np.array([vx, vy, vz])
            vel_err = self._safe(d["Velocity Err"], default=None)

            twist_msg = TwistWithCovarianceStamped()
            twist_msg.header.stamp = stamp
            twist_msg.header.frame_id = "base_link"
            twist_msg.twist.twist.linear.x = vel[0]
            twist_msg.twist.twist.linear.y = vel[1]
            twist_msg.twist.twist.linear.z = vel[2]

            twist_msg.twist.covariance = self._build_twist_cov(velocity_error=vel_err)

            self.sensor_publishers["velocity"].publish(twist_msg)

        if self.is_active("position"):
            mean_range = self._safe(d["Mean range"])

            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = stamp
            pose_msg.header.frame_id = "base_link"
            pose_msg.pose.pose.position.z = mean_range
            pose_msg.pose.pose.orientation.w = 1.0

            pose_msg.pose.covariance = self._build_pose_cov()

            self.sensor_publishers["position"].publish(pose_msg)

    def _publish_dummy(self):
        stamp = self.get_clock().now().to_msg()

        if self.is_active("velocity"):
            twist_msg = TwistWithCovarianceStamped()
            twist_msg.header.stamp = stamp
            twist_msg.header.frame_id = "base_link"
            twist_msg.twist.twist.linear.x = 0.05
            twist_msg.twist.covariance = self._build_twist_cov()
            self.sensor_publishers["velocity"].publish(twist_msg)

        if self.is_active("position"):
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = stamp
            pose_msg.header.frame_id = "base_link"
            pose_msg.pose.pose.position.z = 1.5
            pose_msg.pose.pose.orientation.w = 1.0

            pose_msg.pose.covariance = self._build_pose_cov()

            self.sensor_publishers["position"].publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DVL()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
