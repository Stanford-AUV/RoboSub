import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from transforms3d.euler import quat2euler

MIN = -5
MAX = 5


class LocalizationPlot(Node):
    def __init__(self):
        super().__init__("localization_plot")

        # Create subscriber for odometry
        self.odom_sub = self.create_subscription(
            Odometry, "/odometry/filtered", self.odom_callback, 10
        )

        # Initialize plot
        plt.ion()  # Enable interactive mode
        self.fig = plt.figure(figsize=(15, 10))

        # Create subplots
        self.ax_3d = self.fig.add_subplot(241, projection="3d")
        self.ax_x = self.fig.add_subplot(242)
        self.ax_y = self.fig.add_subplot(243)
        self.ax_z = self.fig.add_subplot(244)
        self.ax_roll = self.fig.add_subplot(245)
        self.ax_pitch = self.fig.add_subplot(246)
        self.ax_yaw = self.fig.add_subplot(247)

        # Set titles
        self.ax_3d.set_title("Robot Position Trajectory")
        self.ax_x.set_title("X Position vs Time")
        self.ax_y.set_title("Y Position vs Time")
        self.ax_z.set_title("Z Position vs Time")
        self.ax_roll.set_title("Roll Angle vs Time")
        self.ax_pitch.set_title("Pitch Angle vs Time")
        self.ax_yaw.set_title("Yaw Angle vs Time")

        # Initialize position and time history
        self.x_history = []
        self.y_history = []
        self.z_history = []
        self.time_history = []
        self.start_time = self.get_clock().now()
        self.roll_history = []
        self.pitch_history = []
        self.yaw_history = []

        # Plot update rate (in seconds)
        self.update_period = 0.1
        self.last_plot_time = self.get_clock().now()

    def odom_callback(self, msg):
        # Extract position from odometry message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        # Extract orientation quaternion and convert to euler angles
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        # Convert quaternion to euler angles (roll, pitch, yaw)
        roll, pitch, yaw = quat2euler([qw, qx, qy, qz])

        # Calculate time since start
        current_time = self.get_clock().now()
        time_sec = (current_time - self.start_time).nanoseconds * 1e-9

        # Append to history
        self.x_history.append(x)
        self.y_history.append(y)
        self.z_history.append(z)
        self.time_history.append(time_sec)
        self.roll_history.append(roll)
        self.pitch_history.append(pitch)
        self.yaw_history.append(yaw)

        # Update plot periodically
        if (
            current_time - self.last_plot_time
        ).nanoseconds * 1e-9 >= self.update_period:
            self.update_plot()
            self.last_plot_time = current_time

    def update_plot(self):
        # Clear all plots
        self.ax_3d.cla()
        self.ax_x.cla()
        self.ax_y.cla()
        self.ax_z.cla()
        self.ax_roll.cla()
        self.ax_pitch.cla()
        self.ax_yaw.cla()

        # Update 3D trajectory plot
        self.ax_3d.plot(self.x_history, self.y_history, self.z_history, "b-")
        if self.x_history:
            self.ax_3d.scatter(
                self.x_history[-1],
                self.y_history[-1],
                self.z_history[-1],
                color="red",
                marker="o",
            )
        self.ax_3d.set_xlabel("X")
        self.ax_3d.set_ylabel("Y")
        self.ax_3d.set_zlabel("Z")
        self.ax_3d.set_title("Robot Position Trajectory")
        # Set fixed limits for 3D plot
        self.ax_3d.set_xlim([MIN, MAX])
        self.ax_3d.set_ylim([MIN, MAX])
        self.ax_3d.set_zlim([MIN, MAX])

        # Update individual position plots
        self.ax_x.plot(self.time_history, self.x_history, "r-")
        self.ax_x.set_xlabel("Time (s)")
        self.ax_x.set_ylabel("X Position")
        self.ax_x.set_ylim([MIN, MAX])
        self.ax_x.grid(True)

        self.ax_y.plot(self.time_history, self.y_history, "g-")
        self.ax_y.set_xlabel("Time (s)")
        self.ax_y.set_ylabel("Y Position")
        self.ax_y.set_ylim([MIN, MAX])
        self.ax_y.grid(True)

        self.ax_z.plot(self.time_history, self.z_history, "b-")
        self.ax_z.set_xlabel("Time (s)")
        self.ax_z.set_ylabel("Z Position")
        self.ax_z.set_ylim([MIN, MAX])
        self.ax_z.grid(True)

        # Update orientation plots
        self.ax_roll.plot(self.time_history, self.roll_history, "m-")
        self.ax_roll.set_xlabel("Time (s)")
        self.ax_roll.set_ylabel("Roll (rad)")
        self.ax_roll.set_ylim([-3.14, 3.14])
        self.ax_roll.grid(True)

        self.ax_pitch.plot(self.time_history, self.pitch_history, "c-")
        self.ax_pitch.set_xlabel("Time (s)")
        self.ax_pitch.set_ylabel("Pitch (rad)")
        self.ax_pitch.set_ylim([-3.14, 3.14])
        self.ax_pitch.grid(True)

        self.ax_yaw.plot(self.time_history, self.yaw_history, "y-")
        self.ax_yaw.set_xlabel("Time (s)")
        self.ax_yaw.set_ylabel("Yaw (rad)")
        self.ax_yaw.set_ylim([-3.14, 3.14])
        self.ax_yaw.grid(True)

        # Adjust layout and update display
        plt.tight_layout()
        plt.draw()
        plt.pause(0.001)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationPlot()

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
