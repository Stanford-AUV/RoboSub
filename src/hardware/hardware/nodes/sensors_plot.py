import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry

import matplotlib

matplotlib.use("Agg")  # headless: write PNG to disk, no display forwarding needed
import matplotlib.pyplot as plt
import math
import numpy as np
from scipy.spatial.transform import Rotation as R


# curr pitch y = yaw z
# curr roll x = pitch y
# curr yaw z = roll x
class SensorsPlot(Node):
    def __init__(self):
        super().__init__("sensors_plot")

        self.accel_sub = self.create_subscription(
            Imu, "/accel", self.accel_callback, 10
        )
        self.angular_sub = self.create_subscription(
            TwistWithCovarianceStamped, "/angular", self.angular_callback, 10
        )
        self.velocity_sub = self.create_subscription(
            TwistWithCovarianceStamped, "/velocity", self.velocity_callback, 10
        )

        self.rotation_sub = self.create_subscription(
            Odometry, "/odometry/filtered", self.rotation_callback, 10
        )

        self.fig = plt.figure(figsize=(12, 8))

        self.ax_imu_accel = self.fig.add_subplot(221)
        self.ax_imu_gyro = self.fig.add_subplot(222)
        self.ax_dvl_vel = self.fig.add_subplot(223)
        self.ax_imu_rot = self.fig.add_subplot(224)

        self.ax_imu_accel.set_title("IMU Acceleration")
        self.ax_imu_gyro.set_title("IMU Angular Velocity")
        self.ax_dvl_vel.set_title("DVL Velocity")
        self.ax_imu_rot.set_title("EKF Rotation (/odometry/filtered)")

        self.start_time = self.get_clock().now()

        self.accel_time = []
        self.accel_x_history = []
        self.accel_y_history = []
        self.accel_z_history = []

        self.gyro_time = []
        self.gyro_x_history = []
        self.gyro_y_history = []
        self.gyro_z_history = []

        self.vel_time = []
        self.vel_x_history = []
        self.vel_y_history = []
        self.vel_z_history = []

        self.rot_time = []
        self.rot_x_history = []
        self.rot_y_history = []
        self.rot_z_history = []

        self.update_period = 0.1
        self.last_plot_time = self.get_clock().now()

    def _elapsed(self):
        return (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

    def _maybe_update_plot(self):
        now = self.get_clock().now()
        if (now - self.last_plot_time).nanoseconds * 1e-9 >= self.update_period:
            self.update_plot()
            self.last_plot_time = now

    def accel_callback(self, msg: Imu):
        self.accel_time.append(self._elapsed())
        self.accel_x_history.append(msg.linear_acceleration.x)
        self.accel_y_history.append(msg.linear_acceleration.y)
        self.accel_z_history.append(msg.linear_acceleration.z)
        self._maybe_update_plot()

    def angular_callback(self, msg: TwistWithCovarianceStamped):
        self.gyro_time.append(self._elapsed())
        self.gyro_x_history.append(msg.twist.twist.angular.x)
        self.gyro_y_history.append(msg.twist.twist.angular.y)
        self.gyro_z_history.append(msg.twist.twist.angular.z)
        self._maybe_update_plot()

    def velocity_callback(self, msg: TwistWithCovarianceStamped):
        self.vel_time.append(self._elapsed())
        self.vel_x_history.append(msg.twist.twist.linear.x)
        self.vel_y_history.append(msg.twist.twist.linear.y)
        self.vel_z_history.append(msg.twist.twist.linear.z)
        self._maybe_update_plot()

    def rotation_callback(self, msg: Odometry):
        self.rot_time.append(self._elapsed())
        quat = msg.pose.pose.orientation

        # w, x, y, z = self.rotate_quaternion(quat.w, quat.x, quat.y, quat.z)

        r, p, y = self.quaternion_to_rpy(quat.w, quat.x, quat.y, quat.z)

        self.rot_x_history.append(r)
        self.rot_y_history.append(p)
        self.rot_z_history.append(y)
        self._maybe_update_plot()

    def rotate_quaternion(self, w, x, y, z):  # x, y, z, w
        original_data = [x, y, z, w]
        q_original = R.from_quat(original_data)

        r_base = R.from_quat([0.0, 0.7071, 0.0, -0.7071])  # 90 deg around Y
        r_flip = R.from_quat([0.0, 0.0, 1.0, 0.0])
        r_change = r_flip * r_base

        q_new = r_change * q_original * r_change.inv()
        quat_array = q_new.as_quat()
        x, y, z, w = quat_array

        return w, x, y, z

    def quaternion_to_rpy(self, q_w, q_x, q_y, q_z):
        # Roll (x-axis)
        sinr_cosp = 2 * (q_w * q_x + q_y * q_z)
        cosr_cosp = 1 - 2 * (q_x**2 + q_y**2)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis)
        pitch = 2 * (q_w * q_y - q_z * q_x)
        if abs(pitch) >= 1:
            pitch = math.copysign(math.pi / 2, pitch)
        else:
            pitch = math.asin(pitch)

        # Yaw (z-axis)
        siny_cosp = 2 * (q_w * q_z + q_x * q_y)
        cosy_cosp = 1 - 2 * (q_y**2 + q_z**2)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def update_plot(self):
        # Clear all plots
        self.ax_imu_accel.cla()
        self.ax_imu_gyro.cla()
        self.ax_dvl_vel.cla()
        self.ax_imu_rot.cla()

        self.ax_imu_accel.plot(self.accel_time, self.accel_x_history, "r-", label="X")
        self.ax_imu_accel.plot(self.accel_time, self.accel_y_history, "g-", label="Y")
        self.ax_imu_accel.plot(self.accel_time, self.accel_z_history, "b-", label="Z")
        self.ax_imu_accel.set_xlabel("Time (s)")
        self.ax_imu_accel.set_ylabel("Acceleration (m/s²)")
        self.ax_imu_accel.grid(True)
        self.ax_imu_accel.legend()

        self.ax_imu_gyro.plot(self.gyro_time, self.gyro_x_history, "r-", label="X")
        self.ax_imu_gyro.plot(self.gyro_time, self.gyro_y_history, "g-", label="Y")
        self.ax_imu_gyro.plot(self.gyro_time, self.gyro_z_history, "b-", label="Z")
        self.ax_imu_gyro.set_xlabel("Time (s)")
        self.ax_imu_gyro.set_ylabel("Angular Velocity (rad/s)")
        self.ax_imu_gyro.grid(True)
        self.ax_imu_gyro.legend()

        self.ax_dvl_vel.plot(self.vel_time, self.vel_x_history, "r-", label="X")
        self.ax_dvl_vel.plot(self.vel_time, self.vel_y_history, "g-", label="Y")
        self.ax_dvl_vel.plot(self.vel_time, self.vel_z_history, "b-", label="Z")
        self.ax_dvl_vel.set_xlabel("Time (s)")
        self.ax_dvl_vel.set_ylabel("Velocity (m/s)")
        self.ax_dvl_vel.grid(True)
        self.ax_dvl_vel.legend()

        # Unwrap the angles to remove the jumps before plotting
        smooth_roll = np.unwrap(self.rot_x_history)
        smooth_pitch = np.unwrap(self.rot_y_history)
        smooth_yaw = np.unwrap(self.rot_z_history)

        self.ax_imu_rot.plot(self.rot_time, smooth_roll, "r-", label="ROLL")
        self.ax_imu_rot.plot(self.rot_time, smooth_pitch, "g-", label="PITCH")
        self.ax_imu_rot.plot(self.rot_time, smooth_yaw, "b-", label="YAW")
        self.ax_imu_rot.set_xlabel("Time (s)")
        self.ax_imu_rot.set_ylabel("Rotation (rad)")
        self.ax_imu_rot.grid(True)
        self.ax_imu_rot.legend()

        # Adjust layout and update display
        plt.tight_layout()
        self.fig.savefig("/workspaces/RoboSub/sensors_plot.png")


def main(args=None):
    rclpy.init(args=args)
    node = SensorsPlot()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        plt.close("all")


if __name__ == "__main__":
    main()
