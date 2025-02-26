import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt


class LocalizationPlot(Node):
    def __init__(self):
        super().__init__("localization_plot")

        # Create subscriber for odometry
        self.odom_sub = self.create_subscription(
            Odometry, "/odometry/filtered", self.odom_callback, 10
        )

        # Initialize plot
        plt.ion()  # Enable interactive mode
        self.fig = plt.figure(figsize=(12, 8))

        # Create subplots
        self.ax_3d = self.fig.add_subplot(221, projection="3d")
        self.ax_x = self.fig.add_subplot(222)
        self.ax_y = self.fig.add_subplot(223)
        self.ax_z = self.fig.add_subplot(224)

        # Set titles
        self.ax_3d.set_title("Robot Position Trajectory")
        self.ax_x.set_title("X Position vs Time")
        self.ax_y.set_title("Y Position vs Time")
        self.ax_z.set_title("Z Position vs Time")

        # Initialize position and time history
        self.x_history = []
        self.y_history = []
        self.z_history = []
        self.time_history = []
        self.start_time = self.get_clock().now()

        # Plot update rate (in seconds)
        self.update_period = 0.1
        self.last_plot_time = self.get_clock().now()

    def odom_callback(self, msg):
        # Extract position from odometry message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        # Calculate time since start
        current_time = self.get_clock().now()
        time_sec = (current_time - self.start_time).nanoseconds * 1e-9

        # Append to history
        self.x_history.append(x)
        self.y_history.append(y)
        self.z_history.append(z)
        self.time_history.append(time_sec)

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

        # Update individual position plots
        self.ax_x.plot(self.time_history, self.x_history, "r-")
        self.ax_x.set_xlabel("Time (s)")
        self.ax_x.set_ylabel("X Position")
        self.ax_x.grid(True)

        self.ax_y.plot(self.time_history, self.y_history, "g-")
        self.ax_y.set_xlabel("Time (s)")
        self.ax_y.set_ylabel("Y Position")
        self.ax_y.grid(True)

        self.ax_z.plot(self.time_history, self.z_history, "b-")
        self.ax_z.set_xlabel("Time (s)")
        self.ax_z.set_ylabel("Z Position")
        self.ax_z.grid(True)

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
