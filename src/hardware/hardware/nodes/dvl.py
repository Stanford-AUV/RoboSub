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
            "position": self.create_publisher(
                PoseWithCovarianceStamped, "/position", 10
            ),
            "velocity": self.create_publisher(
                TwistWithCovarianceStamped, "/velocity", 10
            ),
        }

        self._latest_data = None

        # Body angular velocity for lever-arm compensation: a DVL mounted at
        # offset r from base_link measures v_base + omega x r; we subtract the
        # rotation-induced part. Requires dvl_0.sensor_pos_in_base in
        # sensors.yaml (x,y,z meters); inactive (zero correction) until set.
        self._omega = np.zeros(3)
        self._lever_arm = (
            np.array(self.sensor_pos_in_base, dtype=float)
            if self.sensor_pos_in_base
            else None
        )
        self.create_subscription(
            TwistWithCovarianceStamped, "/angular", self._angular_callback, 10
        )

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
        preferred = ["/dev/ttyUSB_dvl"]
        fallback = glob.glob("/dev/ttyUSB[0-9]*") + glob.glob("/dev/ttyACM[0-9]*")
        possible_ports = preferred + [p for p in fallback if p not in preferred]

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

    def _angular_callback(self, msg: TwistWithCovarianceStamped):
        a = msg.twist.twist.angular
        self._omega = np.array([a.x, a.y, a.z])

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
                var = velocity_error**2
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
            vx = d["Velocity X"]
            vy = d["Velocity Y"]
            vz = d["Velocity Z"]

            # The Wayfinder reports NaN velocity when it has no bottom lock
            # (out of water, below blanking range, too far, or bad return).
            # Publishing a masked 0.0 tells the EKF "confidently stationary"
            # with a small covariance, which poisons the filter. Skip instead.
            if any(np.isnan(v) for v in (vx, vy, vz)):
                self.get_logger().warn(
                    "DVL: no bottom lock (velocity NaN) -> not publishing. "
                    f"coord={d.get('Coordinate')} mean_range={d.get('Mean range')} "
                    f"beams(m)=[{d.get('Beam 1 range')}, {d.get('Beam 2 range')}, "
                    f"{d.get('Beam 3 range')}, {d.get('Beam 4 range')}]",
                    throttle_duration_sec=2.0,
                )
                self._latest_data = None
                return

            self.get_logger().info(
                f"DVL lock: raw v=[{vx:.3f}, {vy:.3f}, {vz:.3f}] "
                f"err={d.get('Velocity Err')} mean_range={d.get('Mean range')} "
                f"coord={d.get('Coordinate')}",
                throttle_duration_sec=1.0,
            )

            vel = self.R_sensor_to_base @ np.array([vx, vy, vz])
            if self._lever_arm is not None:
                vel = vel - np.cross(self._omega, self._lever_arm)
            vel_err = self._safe(d["Velocity Err"], default=None)

            twist_msg = TwistWithCovarianceStamped()
            twist_msg.header.stamp = stamp
            twist_msg.header.frame_id = "base_link"
            twist_msg.twist.twist.linear.x = vel[0]
            twist_msg.twist.twist.linear.y = vel[1]
            twist_msg.twist.twist.linear.z = vel[2]

            twist_msg.twist.covariance = self._build_twist_cov(
                velocity_error=vel_err
            )

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

        self._latest_data = None

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
