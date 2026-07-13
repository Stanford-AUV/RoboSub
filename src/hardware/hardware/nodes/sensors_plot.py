import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped, AccelWithCovarianceStamped
from nav_msgs.msg import Odometry

import matplotlib

matplotlib.use("Agg")  # headless: write PNG to disk, no display forwarding needed
import matplotlib.pyplot as plt
import math
import os
import numpy as np


class SensorsPlot(Node):
    """Plot the pieces of the localization puzzle: what we FEED the EKF (IMU
    absolute rotation, DVL velocity) next to what the EKF PUTS OUT (rotation,
    velocity, position, acceleration)."""

    # savefig blocks the executor for ~1-2 s; queues must hold that much
    # backlog at 60-400 Hz or DDS silently drops messages and the plot
    # aliases (flat holds turn into ramps). Keep this generous.
    QUEUE_DEPTH = 1000

    def __init__(self):
        super().__init__("sensors_plot")

        # ---- EKF INPUTS ----
        # Absolute orientation measurement we feed the EKF (imu.py output).
        self.imu_in_sub = self.create_subscription(
            Imu, "/imu/orientation", self.imu_in_callback, self.QUEUE_DEPTH
        )
        # DVL body velocity.
        self.velocity_sub = self.create_subscription(
            TwistWithCovarianceStamped, "/velocity", self.velocity_callback,
            self.QUEUE_DEPTH,
        )

        # ---- EKF OUTPUTS ----
        self.odom_sub = self.create_subscription(
            Odometry, "/odometry/filtered", self.odom_callback, self.QUEUE_DEPTH
        )
        self.ekf_accel_sub = self.create_subscription(
            AccelWithCovarianceStamped, "/accel/filtered", self.ekf_accel_callback,
            self.QUEUE_DEPTH,
        )
        # Desired position setpoint (planning -> control).
        self.desired_sub = self.create_subscription(
            Odometry, "/desired/pose", self.desired_callback, self.QUEUE_DEPTH
        )

        self.fig = plt.figure(figsize=(24, 8))

        # Top row: INPUTS + EKF rotation.  Bottom row: EKF outputs + desired pos.
        self.ax_imu_in = self.fig.add_subplot(241)
        self.ax_dvl_vel = self.fig.add_subplot(242)
        self.ax_ekf_rot = self.fig.add_subplot(243)
        self.ax_ekf_vel = self.fig.add_subplot(245)
        self.ax_ekf_pos = self.fig.add_subplot(246)
        self.ax_ekf_accel = self.fig.add_subplot(247)
        self.ax_des_pos = self.fig.add_subplot(248)

        self.imu_in_time = []
        self.imu_in_roll_history = []
        self.imu_in_pitch_history = []
        self.imu_in_yaw_history = []

        self.vel_time = []
        self.vel_x_history = []
        self.vel_y_history = []
        self.vel_z_history = []

        self.rot_time = []
        self.rot_x_history = []
        self.rot_y_history = []
        self.rot_z_history = []

        self.ekf_vel_time = []
        self.ekf_vel_x_history = []
        self.ekf_vel_y_history = []
        self.ekf_vel_z_history = []

        self.ekf_pos_time = []
        self.ekf_pos_x_history = []
        self.ekf_pos_y_history = []
        self.ekf_pos_z_history = []

        self.ekf_accel_time = []
        self.ekf_accel_x_history = []
        self.ekf_accel_y_history = []
        self.ekf_accel_z_history = []

        self.des_pos_time = []
        self.des_pos_x_history = []
        self.des_pos_y_history = []
        self.des_pos_z_history = []

        # Redraw on a timer, NOT in the message callbacks: savefig takes
        # ~1-2 s and doing it inline starved the subscriptions, dropping
        # ~99% of samples and aliasing the curves.
        self._t0 = None  # first header stamp seen; time axis is relative to it
        self.create_timer(2.0, self.update_plot)

    def _stamp(self, msg):
        """Seconds since start-of-run, from the message HEADER stamp (true
        sample time), not arrival time - arrival time smears samples that
        sat in the queue while the plotter was busy saving."""
        s = msg.header.stamp
        t = s.sec + s.nanosec * 1e-9
        if t == 0.0:  # unstamped publisher: fall back to node clock
            t = self.get_clock().now().nanoseconds * 1e-9
        if self._t0 is None:
            self._t0 = t
        return t - self._t0

    def imu_in_callback(self, msg: Imu):
        self.imu_in_time.append(self._stamp(msg))
        q = msg.orientation
        r, p, y = self.quaternion_to_rpy(q.w, q.x, q.y, q.z)
        self.imu_in_roll_history.append(r)
        self.imu_in_pitch_history.append(p)
        self.imu_in_yaw_history.append(y)

    def velocity_callback(self, msg: TwistWithCovarianceStamped):
        self.vel_time.append(self._stamp(msg))
        self.vel_x_history.append(msg.twist.twist.linear.x)
        self.vel_y_history.append(msg.twist.twist.linear.y)
        self.vel_z_history.append(msg.twist.twist.linear.z)

    def odom_callback(self, msg: Odometry):
        t = self._stamp(msg)
        self.rot_time.append(t)
        quat = msg.pose.pose.orientation
        r, p, y = self.quaternion_to_rpy(quat.w, quat.x, quat.y, quat.z)
        self.rot_x_history.append(r)
        self.rot_y_history.append(p)
        self.rot_z_history.append(y)

        self.ekf_vel_time.append(t)
        self.ekf_vel_x_history.append(msg.twist.twist.linear.x)
        self.ekf_vel_y_history.append(msg.twist.twist.linear.y)
        self.ekf_vel_z_history.append(msg.twist.twist.linear.z)

        pos = msg.pose.pose.position
        self.ekf_pos_time.append(t)
        self.ekf_pos_x_history.append(pos.x)
        self.ekf_pos_y_history.append(pos.y)
        self.ekf_pos_z_history.append(pos.z)

    def ekf_accel_callback(self, msg: AccelWithCovarianceStamped):
        self.ekf_accel_time.append(self._stamp(msg))
        self.ekf_accel_x_history.append(msg.accel.accel.linear.x)
        self.ekf_accel_y_history.append(msg.accel.accel.linear.y)
        self.ekf_accel_z_history.append(msg.accel.accel.linear.z)

    def desired_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        self.des_pos_time.append(self._stamp(msg))
        self.des_pos_x_history.append(pos.x)
        self.des_pos_y_history.append(pos.y)
        self.des_pos_z_history.append(pos.z)

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
        for ax in (
            self.ax_imu_in,
            self.ax_dvl_vel,
            self.ax_ekf_rot,
            self.ax_ekf_vel,
            self.ax_ekf_pos,
            self.ax_ekf_accel,
            self.ax_des_pos,
        ):
            ax.cla()

        # ---- INPUT: IMU absolute rotation fed to the EKF ----
        imu_roll = np.unwrap(self.imu_in_roll_history)
        imu_pitch = np.unwrap(self.imu_in_pitch_history)
        imu_yaw = np.unwrap(self.imu_in_yaw_history)
        self.ax_imu_in.set_title("IMU Rotation IN (/imu/orientation)")
        self.ax_imu_in.plot(self.imu_in_time, imu_roll, "r-", label="ROLL")
        self.ax_imu_in.plot(self.imu_in_time, imu_pitch, "g-", label="PITCH")
        self.ax_imu_in.plot(self.imu_in_time, imu_yaw, "b-", label="YAW")
        self.ax_imu_in.set_xlabel("Time (s)")
        self.ax_imu_in.set_ylabel("Rotation (rad)")
        self.ax_imu_in.grid(True)
        self.ax_imu_in.legend()

        # ---- INPUT: DVL velocity ----
        self.ax_dvl_vel.set_title("DVL Velocity (/velocity)")
        self.ax_dvl_vel.plot(self.vel_time, self.vel_x_history, "r-", label="X")
        self.ax_dvl_vel.plot(self.vel_time, self.vel_y_history, "g-", label="Y")
        self.ax_dvl_vel.plot(self.vel_time, self.vel_z_history, "b-", label="Z")
        self.ax_dvl_vel.set_xlabel("Time (s)")
        self.ax_dvl_vel.set_ylabel("Velocity (m/s)")
        self.ax_dvl_vel.grid(True)
        self.ax_dvl_vel.legend()

        # ---- OUTPUT: EKF rotation ----
        smooth_roll = np.unwrap(self.rot_x_history)
        smooth_pitch = np.unwrap(self.rot_y_history)
        smooth_yaw = np.unwrap(self.rot_z_history)
        self.ax_ekf_rot.set_title("EKF Rotation (/odometry/filtered)")
        self.ax_ekf_rot.plot(self.rot_time, smooth_roll, "r-", label="ROLL")
        self.ax_ekf_rot.plot(self.rot_time, smooth_pitch, "g-", label="PITCH")
        self.ax_ekf_rot.plot(self.rot_time, smooth_yaw, "b-", label="YAW")
        self.ax_ekf_rot.set_xlabel("Time (s)")
        self.ax_ekf_rot.set_ylabel("Rotation (rad)")
        self.ax_ekf_rot.grid(True)
        self.ax_ekf_rot.legend()

        # ---- OUTPUT: EKF velocity ----
        self.ax_ekf_vel.set_title("EKF Velocity (/odometry/filtered)")
        self.ax_ekf_vel.plot(self.ekf_vel_time, self.ekf_vel_x_history, "r-", label="X")
        self.ax_ekf_vel.plot(self.ekf_vel_time, self.ekf_vel_y_history, "g-", label="Y")
        self.ax_ekf_vel.plot(self.ekf_vel_time, self.ekf_vel_z_history, "b-", label="Z")
        self.ax_ekf_vel.set_xlabel("Time (s)")
        self.ax_ekf_vel.set_ylabel("Velocity (m/s)")
        self.ax_ekf_vel.grid(True)
        self.ax_ekf_vel.legend()

        # ---- OUTPUT: EKF position ----
        # Z is pinned by depth; X/Y dead-reckon from DVL velocity and drift
        # with no absolute horizontal fix.
        self.ax_ekf_pos.set_title("EKF Position (/odometry/filtered)")
        self.ax_ekf_pos.plot(self.ekf_pos_time, self.ekf_pos_x_history, "r-", label="X")
        self.ax_ekf_pos.plot(self.ekf_pos_time, self.ekf_pos_y_history, "g-", label="Y")
        self.ax_ekf_pos.plot(self.ekf_pos_time, self.ekf_pos_z_history, "b-", label="Z")
        self.ax_ekf_pos.set_xlabel("Time (s)")
        self.ax_ekf_pos.set_ylabel("Position (m)")
        self.ax_ekf_pos.grid(True)
        self.ax_ekf_pos.legend()

        # ---- OUTPUT: EKF acceleration ----
        self.ax_ekf_accel.set_title("EKF Acceleration (/accel/filtered)")
        self.ax_ekf_accel.plot(self.ekf_accel_time, self.ekf_accel_x_history, "r-", label="X")
        self.ax_ekf_accel.plot(self.ekf_accel_time, self.ekf_accel_y_history, "g-", label="Y")
        self.ax_ekf_accel.plot(self.ekf_accel_time, self.ekf_accel_z_history, "b-", label="Z")
        self.ax_ekf_accel.set_xlabel("Time (s)")
        self.ax_ekf_accel.set_ylabel("Acceleration (m/s²)")
        self.ax_ekf_accel.grid(True)
        self.ax_ekf_accel.legend()

        # ---- DESIRED: position setpoint (/desired/pose) ----
        self.ax_des_pos.set_title("Desired Position (/desired/pose)")
        self.ax_des_pos.plot(self.des_pos_time, self.des_pos_x_history, "r-", label="X")
        self.ax_des_pos.plot(self.des_pos_time, self.des_pos_y_history, "g-", label="Y")
        self.ax_des_pos.plot(self.des_pos_time, self.des_pos_z_history, "b-", label="Z")
        self.ax_des_pos.set_xlabel("Time (s)")
        self.ax_des_pos.set_ylabel("Position (m)")
        self.ax_des_pos.grid(True)
        self.ax_des_pos.legend()

        plt.tight_layout()
        # Land the PNG in the repo root, NOT the launch CWD. ros2 launch starts
        # nodes from $HOME, so the old cwd-relative path wrote to ~/sensors_plot.png
        # where nobody was looking. Prefer $ROBOSUB_DIR, then ~/RoboSub, then cwd.
        out_dir = os.environ.get("ROBOSUB_DIR") or os.path.expanduser("~/RoboSub")
        if not os.path.isdir(out_dir):
            out_dir = os.getcwd()
        out_path = os.path.join(out_dir, "sensors_plot.png")
        self.fig.savefig(out_path)
        if not getattr(self, "_logged_out_path", False):
            self.get_logger().info(f"writing sensors plot to {out_path}")
            self._logged_out_path = True


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
