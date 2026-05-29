"""
Fake localizer for testing the full pipeline without hardware.

Subscribes to /wrench, applies simple physics (F=ma + drag), and publishes
fake odometry on /odometry/filtered at 50 Hz.

Tune MASS and DRAG to match rough sub dynamics.
Force from pid_control arrives in body frame; we rotate to world frame before integrating.
"""

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped

MASS = 15.0          # kg
INERTIA = np.array([0.5, 0.5, 1.0])   # kg·m² (roll, pitch, yaw)
DRAG_LINEAR = 8.0    # N·s/m  — tune so terminal velocity feels realistic
DRAG_ANGULAR = 1.5   # N·m·s/rad


class FakeLocalizer(Node):
    def __init__(self):
        super().__init__("fake_localizer")

        self.position = np.zeros(3)
        self.velocity = np.zeros(3)          # world frame
        self.orientation = Rotation.identity()
        self.angular_velocity = np.zeros(3)  # body frame

        self.force_body = np.zeros(3)
        self.torque_body = np.zeros(3)

        self.pub = self.create_publisher(Odometry, "/odometry/filtered", 10)
        self.create_subscription(WrenchStamped, "/wrench", self._wrench_cb, 10)

        self.dt = 1.0 / 50.0
        self.create_timer(self.dt, self._update)
        self.get_logger().info("FakeLocalizer started at origin (0,0,0).")

    def _wrench_cb(self, msg: WrenchStamped):
        self.force_body = np.array([
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z,
        ])
        self.torque_body = np.array([
            msg.wrench.torque.x,
            msg.wrench.torque.y,
            msg.wrench.torque.z,
        ])

    def _update(self):
        dt = self.dt

        # Rotate body-frame force to world frame
        force_world = self.orientation.apply(self.force_body)

        # Linear: a = (F - drag*v) / m
        accel = (force_world - DRAG_LINEAR * self.velocity) / MASS
        self.velocity += accel * dt
        self.position += self.velocity * dt

        # Angular: α = (τ - drag*ω) / I  (body frame)
        angular_accel = (self.torque_body - DRAG_ANGULAR * self.angular_velocity) / INERTIA
        self.angular_velocity += angular_accel * dt

        # Integrate orientation
        dtheta = self.angular_velocity * dt
        if np.linalg.norm(dtheta) > 1e-10:
            self.orientation = self.orientation * Rotation.from_rotvec(dtheta)

        # Publish
        q = self.orientation.as_quat()  # [x, y, z, w]
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.child_frame_id = "base_link"

        msg.pose.pose.position.x = float(self.position[0])
        msg.pose.pose.position.y = float(self.position[1])
        msg.pose.pose.position.z = float(self.position[2])
        msg.pose.pose.orientation.x = float(q[0])
        msg.pose.pose.orientation.y = float(q[1])
        msg.pose.pose.orientation.z = float(q[2])
        msg.pose.pose.orientation.w = float(q[3])

        # Velocity in world frame (matches path_generator spline derivatives)
        msg.twist.twist.linear.x = float(self.velocity[0])
        msg.twist.twist.linear.y = float(self.velocity[1])
        msg.twist.twist.linear.z = float(self.velocity[2])
        msg.twist.twist.angular.x = float(self.angular_velocity[0])
        msg.twist.twist.angular.y = float(self.angular_velocity[1])
        msg.twist.twist.angular.z = float(self.angular_velocity[2])

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeLocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
