import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped

from hardware.utils.dvl_utils.system import OutputData
from hardware.utils.dvl_utils.dvl_connect import Dvl
from hardware.nodes.generic_sensor import GenericSensor
import numpy as np
import glob
import serial

DEBUG_MODE = False

# Huge value for covariance diagonal entries the EKF should ignore
_BIG = 1e9


class DVL(GenericSensor):
    def __init__(self, baudrate=115200):
        super().__init__("dvl_ros_bridge", "dvl_0")

        self._latest_data = None

        if DEBUG_MODE:
            self.get_logger().info("Running in DEBUG_MODE, publishing dummy data.")
            self.timer = self.create_timer(0.1, self._publish_dummy)
            return

        # Connect to DVL
        self.dvl = self.autodetect_dvl_port(baudrate)
        if not self.dvl:
            self.get_logger().error("Failed to connect to DVL.")
            return

        # Register the callback function to process DVL data
        self.dvl.register_ondata_callback(self.update_data)

        # Start pinging
        if not self.dvl.exit_command_mode():
            self.get_logger().error("Failed to start pinging")

    # ------------------------------------------------------------------ #
    #  Hardware connection
    # ------------------------------------------------------------------ #
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

    # ------------------------------------------------------------------ #
    #  DVL hardware callback → store parsed data, then publish
    # ------------------------------------------------------------------ #
    def update_data(self, output_data: OutputData, obj):
        del obj
        if output_data is None:
            return

        settings = output_data.get_settings()
        data_dict = {setting.name: setting.value for setting in settings}

        self._latest_data = data_dict
        self.publish_sensor_data()

    # ------------------------------------------------------------------ #
    #  Helper: safe float extraction
    # ------------------------------------------------------------------ #
    @staticmethod
    def _safe(value, default=0.0):
        return default if np.isnan(value) else float(value)

    # ------------------------------------------------------------------ #
    #  Covariance helpers
    # ------------------------------------------------------------------ #
    def _yaml_cov(self, r, c):
        """Return a single entry from the 3×3 yaml covariance matrix."""
        if self.covariance is not None:
            return float(self.covariance[r][c])
        return 0.0

    def _build_twist_cov(self, velocity_error=None):
        """Build the 6×6 twist covariance (row-major, 36 elements).
        
        Active velocity axes get variance from hardware error (if provided) or yaml.
        Inactive velocity axes get _BIG on diagonal so EKF ignores them.
        Angular axes always get _BIG (DVL doesn't provide angular velocity).
        """
        cov = np.zeros(36)
        
        # Linear velocity (indices 0-2)
        vel_axes = self.get_axes("velocity")  # e.g. [1, 2, 3] for x,y,z
        
        # Start with _BIG on all linear diagonals
        for i in range(3):
            cov[i * 6 + i] = _BIG
            
        # Apply variance to active axes
        if vel_axes:
            if velocity_error is not None:
                # Use hardware-provided error (squared for variance)
                var = velocity_error ** 2
                for axis in vel_axes:
                    idx = axis - 1
                    cov[idx * 6 + idx] = var
            elif self.covariance is not None:
                # Fall back to yaml covariance
                for r in range(3):
                    for c in range(3):
                        if (r + 1) in vel_axes and (c + 1) in vel_axes:
                            cov[r * 6 + c] = float(self.covariance[r][c])
        
        # Angular velocity always ignored
        cov[3 * 6 + 3] = _BIG
        cov[4 * 6 + 4] = _BIG
        cov[5 * 6 + 5] = _BIG
        
        return cov.tolist()

    def _build_pose_cov(self):
        """Build the 6×6 pose covariance (row-major, 36 elements).
        
        Active position axes get the yaml covariance.
        Inactive position axes get _BIG on diagonal so EKF ignores them.
        Rotation block is left at zero (EKF doesn't read it from Pose).
        """
        cov = np.zeros(36)
        
        # Position (indices 0-2 for x,y,z)
        pos_axes = self.get_axes("position")  # e.g. [3] for z only
        
        # Start with _BIG on all position diagonals
        for i in range(3):
            cov[i * 6 + i] = _BIG
            
        # Then apply yaml covariance to active axes
        if self.covariance is not None and pos_axes:
            for axis in pos_axes:
                # Map axis number (1,2,3) to index (0,1,2)
                idx = axis - 1
                cov[idx * 6 + idx] = self._yaml_cov(idx, idx)
        
        # Rotation block stays at zero (indices 3-5)
        
        return cov.tolist()

    # ------------------------------------------------------------------ #
    #  Publish to GenericSensor publishers
    # ------------------------------------------------------------------ #
    def publish_sensor_data(self):
        d = self._latest_data
        if d is None:
            return

        stamp = self.get_clock().now().to_msg()

        # --- velocity (TwistWithCovarianceStamped) ---
        if self.is_active("velocity"):
            vx = self._safe(d["Velocity X"])
            vy = self._safe(d["Velocity Y"])
            vz = self._safe(d["Velocity Z"])
            vel = self.robot_rot @ np.array([vx, vy, vz])
            
            # Get hardware-provided velocity error
            vel_err = self._safe(d["Velocity Err"], default=None)

            twist_msg = TwistWithCovarianceStamped()
            twist_msg.header.stamp = stamp
            twist_msg.header.frame_id = "base_link"
            twist_msg.twist.twist.linear.x = vel[0]
            twist_msg.twist.twist.linear.y = vel[1]
            twist_msg.twist.twist.linear.z = vel[2]

            twist_msg.twist.covariance = self._build_twist_cov(velocity_error=vel_err)

            self.publishers["velocity"].publish(twist_msg)

        # --- position (PoseWithCovarianceStamped) — depth (z) only ---
        if self.is_active("position"):
            mean_range = self._safe(d["Mean range"])

            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = stamp
            pose_msg.header.frame_id = "base_link"
            pose_msg.pose.pose.position.z = mean_range
            pose_msg.pose.pose.orientation.w = 1.0

            pose_msg.pose.covariance = self._build_pose_cov()

            self.publishers["position"].publish(pose_msg)

    # ------------------------------------------------------------------ #
    #  Debug dummy publisher
    # ------------------------------------------------------------------ #
    def _publish_dummy(self):
        stamp = self.get_clock().now().to_msg()

        if self.is_active("velocity"):
            twist_msg = TwistWithCovarianceStamped()
            twist_msg.header.stamp = stamp
            twist_msg.header.frame_id = "base_link"
            twist_msg.twist.twist.linear.x = 0.05

            twist_msg.twist.covariance = self._build_twist_cov()  # No hardware error in debug mode

            self.publishers["velocity"].publish(twist_msg)

        if self.is_active("position"):
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = stamp
            pose_msg.header.frame_id = "base_link"
            pose_msg.pose.pose.position.z = 1.5
            pose_msg.pose.pose.orientation.w = 1.0

            pose_msg.pose.covariance = self._build_pose_cov()

            self.publishers["position"].publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DVL()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
