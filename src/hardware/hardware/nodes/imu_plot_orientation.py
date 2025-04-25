import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import matplotlib
import numpy as np
from transforms3d.euler import quat2euler

matplotlib.use("TkAgg")  # Use a GUI backend
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# correction = np.array([0, 0, -9.80665]) - np.array(
#     [-0.5094561923194577, 0.16367032612896282, -9.79881680978311]
# )
correction = np.array([0, 0, 0])


class SensorsPlot(Node):
    def __init__(self):
        super().__init__("sensors_plot")
        # Subscribe to the IMU topic (ORS publishes under "/imu")
        self.imu_sub = self.create_subscription(Imu, "/imu/data", self.imu_callback, 10)

        # Data for orientation
        self.orientation = None
        self.start_time = self.get_clock().now()

        # Setup interactive plotting
        plt.ion()  # Turn on interactive mode
        self.fig = plt.figure(figsize=(10, 10))
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.ax.set_title("IMU Orientation")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        plt.show(block=False)

        # Create a timer to update the plot regularly
        self.plot_timer = self.create_timer(0.1, self.update_plot)

        # Initialize the unit sphere
        u, v = np.mgrid[0 : 2 * np.pi : 20j, 0 : np.pi : 10j]
        x = np.cos(u) * np.sin(v)
        y = np.sin(u) * np.sin(v)
        z = np.cos(v)
        self.ax.plot_wireframe(x, y, z, color="gray", alpha=0.2)

        # Set axis limits
        self.ax.set_xlim([-1.2, 1.2])
        self.ax.set_ylim([-1.2, 1.2])
        self.ax.set_zlim([-1.2, 1.2])

    def imu_callback(self, msg: Imu):
        # Store quaternion orientation
        self.orientation = [
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
        ]

    def update_plot(self):
        if self.orientation is None:
            return

        # Clear previous arrow
        self.ax.cla()

        # Redraw unit sphere
        u, v = np.mgrid[0 : 2 * np.pi : 20j, 0 : np.pi : 10j]
        x = np.cos(u) * np.sin(v)
        y = np.sin(u) * np.sin(v)
        z = np.cos(v)
        self.ax.plot_wireframe(x, y, z, color="gray", alpha=0.2)

        # Convert quaternion to vector (direction) using rotation matrix
        # We'll use a unit vector pointing in the "forward" direction (1,0,0)
        qw, qx, qy, qz = self.orientation

        # Generate basis vectors from quaternion
        x_dir = [
            1 - 2 * (qy**2 + qz**2),
            2 * (qx * qy + qw * qz),
            2 * (qx * qz - qw * qy),
        ]

        y_dir = [
            2 * (qx * qy - qw * qz),
            1 - 2 * (qx**2 + qz**2),
            2 * (qy * qz + qw * qx),
        ]

        z_dir = [
            2 * (qx * qz + qw * qy),
            2 * (qy * qz - qw * qx),
            1 - 2 * (qx**2 + qy**2),
        ]

        # Draw coordinate axes
        self.ax.quiver(0, 0, 0, x_dir[0], x_dir[1], x_dir[2], color="r", label="X")
        self.ax.quiver(0, 0, 0, y_dir[0], y_dir[1], y_dir[2], color="g", label="Y")
        self.ax.quiver(0, 0, 0, z_dir[0], z_dir[1], z_dir[2], color="b", label="Z")

        # Set labels and limits
        self.ax.set_title("IMU Orientation")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        self.ax.set_xlim([-1.2, 1.2])
        self.ax.set_ylim([-1.2, 1.2])
        self.ax.set_zlim([-1.2, 1.2])
        self.ax.legend()

        # Update plot
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    node = SensorsPlot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.close("all")


if __name__ == "__main__":
    main()
