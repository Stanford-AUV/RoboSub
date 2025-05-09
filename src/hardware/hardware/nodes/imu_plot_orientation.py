import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import matplotlib
import numpy as np
from scipy.spatial.transform import Rotation as R

matplotlib.use("TkAgg")  # Use a GUI backend
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

correction = np.array([0, 0, 0])


class SensorsPlot(Node):
    def __init__(self):
        super().__init__("sensors_plot")
        self.imu_sub = self.create_subscription(Imu, "imu", self.imu_callback, 10)

        self.initial_rotation = None
        self.orientation = None

        plt.ion()
        self.fig = plt.figure(figsize=(10, 10))
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.ax.set_title("IMU Orientation")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")

        # Static sphere
        u, v = np.mgrid[0:2 * np.pi:20j, 0:np.pi:10j]
        x = np.cos(u) * np.sin(v)
        y = np.sin(u) * np.sin(v)
        z = np.cos(v)
        self.ax.plot_wireframe(x, y, z, color="gray", alpha=0.2)

        self.ax.set_xlim([-1.2, 1.2])
        self.ax.set_ylim([-1.2, 1.2])
        self.ax.set_zlim([-1.2, 1.2])

        plt.show(block=False)

        self.plot_timer = self.create_timer(0.1, self.update_plot)

        # For drawing arrows
        self.quiver_X = None
        self.quiver_Y = None
        self.quiver_Z = None

    def imu_callback(self, msg: Imu):
        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        rot = R.from_quat(q)

        if self.initial_rotation is None:
            self.initial_rotation = rot

        self.orientation = rot

    def update_plot(self):
        if self.orientation is None or self.initial_rotation is None:
            return

        # Compute delta rotation
        relative_rotation = self.initial_rotation.inv() * self.orientation
        R_quat = relative_rotation.as_matrix()

        # Axis remapping: new_x = old_z, new_y = old_x, new_z = -old_y
        # T = np.array([
        #     [0, 0, 1],
        #     [1, 0, 0],
        #     [0, 1, 0]
        # ])
        # R_new = T @ R_quat @ T.T
        R_new = R_quat

        # Remove old arrows (if they exist)
        if self.quiver_X:
            self.quiver_X.remove()
            self.quiver_Y.remove()
            self.quiver_Z.remove()

        # Draw new axes
        self.quiver_X = self.ax.quiver(0, 0, 0, *R_new[:, 0], color="r", label="X")
        self.quiver_Y = self.ax.quiver(0, 0, 0, *R_new[:, 1], color="g", label="Y")
        self.quiver_Z = self.ax.quiver(0, 0, 0, *R_new[:, 2], color="b", label="Z")

        self.ax.legend()
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
