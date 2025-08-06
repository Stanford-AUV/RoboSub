"""
Controller Node.

This file implements a ROS2 node that computes a control signal based on the
state of the system and a reference state. The node subscribes to the 'state'
and 'path' topics to receive the current state and reference state, and
publishes the computed control signal to the 'wrench' topic. The control signal
is computed using a given control policy, which is a PID controller in this
case.

Dependencies:
    threading
    control.utils.pid.PID
    control.utils.state.State
    geometry_msgs.msg.WrenchStamped
    nav_msgs.msg.Odometry
    numpy
    rclpy.node.Node

Authors:
    Ali Ahmad
    Khaled Messai

Version:
    1.1.0
"""

import threading

from control.utils.pid import PID
from control.utils.state import State

from geometry_msgs.msg import WrenchStamped
from spatialmath.quaternion import UnitQuaternion
from tf_transformations import euler_from_quaternion

from nav_msgs.msg import Odometry

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy import Parameter


class Controller(Node):
    """
    Node that computes a control signal based on state and reference poses.

    Creates subscribers to the 'state' and 'path' topics and publishes to the
    'wrench' topic. Uses a a given control policy to compute the control
    signal to publish.
    """

    def __init__(self, policy):
        """
        Initialize the controller node.

        We initialize the controller node with subscribers to the state and
        reference topics, and a publisher to the desired_wrench topic. We
        initialize the control policy with given parameters and a lock for
        thread safety.

        Parameters
        ----------
        policy
            The policy to use for the controller.
        """
        super().__init__("controller")

        self.declare_parameter("velocity_only", False)

        self.velocity_only = (
            self.get_parameter("velocity_only").get_parameter_value().bool_value
        )

        self.state_subscription = self.create_subscription(
            Odometry, "/odometry/filtered", self.state_callback, 10
        )
        self.reference_subscription = self.create_subscription(
            Odometry, "waypoint", self.reference_callback, 10
        )
        self.control_publisher = self.create_publisher(WrenchStamped, "wrench", 10)

        self.lock = threading.Lock()

        self.time = self.get_clock().now()

        self.policy = policy

        self.ref_state = None

        self.count = 0

    def reset(self):
        """Reset the controller."""
        self.get_logger.info("Resetting controller...")
        with self.lock:
            self.policy.reset()

    def state_callback(self, msg: Odometry):
        """
        Update the current state and update the control signal.

        Parameters
        ----------
        msg : Odometry
            The message containing the state of the system.
        """
        if self.count >= 10:
            px = msg.pose.pose.position.x
            py = msg.pose.pose.position.y
            pz = msg.pose.pose.position.z
            self.get_logger().info(f"Currently at {px:.3f}, {py:.3f}, {pz:.3f}")

            q = msg.pose.pose.orientation
            quat = [q.x, q.y, q.z, q.w]
            roll, pitch, yaw = euler_from_quaternion(quat)
            self.get_logger().info(
                f"Orientation (r,p,y): roll={roll:.3f}, pitch={pitch:.3f}, yaw={yaw:.3f}"
            )

            if self.ref_state is None:
                self.get_logger().info("Waiting for target state...")
            else:
                self.get_logger().info(f"Target state: {self.ref_state.position} {self.ref_state.orientation}")
            
            self.count = 0
        else:
            self.count += 1

        cur_state = State.from_odometry_msg(msg)
        with self.lock:
            ref_state = self.ref_state
            if ref_state is not None:
                ref_state = ref_state.copy()
        self.update(cur_state, ref_state) if ref_state is not None else None

    def reference_callback(self, msg: Odometry):
        """
        Update the reference state.

        Parameters
        ----------
        msg : Odometry
            The message containing the reference state.
        """
        self.get_logger().info(f"Received state {msg}")
        # raise ValueError()
        with self.lock:
            self.ref_state = State.from_odometry_msg(msg)
            self.get_logger().info("Reference state updated")

    def update(self, cur_state, ref_state):
        """Update the control signal and publish to the wrench topic."""
        newTime = self.get_clock().now()
        dt = (newTime - self.time).nanoseconds / 1e9
        self.time = newTime
        wrench = self.policy.update(cur_state, ref_state, dt, self.velocity_only)
        wrench.header.stamp = self.time.to_msg()
        # wrench.wrench.force.z = 0.2
        # wrench = WrenchStamped()
        # wrench.wrench.force.z = z
        if self.count >= 9:
            self.get_logger().info(f"Publishing wrench {wrench}") 
        self.control_publisher.publish(wrench)


def main(args=None):
    """Set up and spin the Controller node."""
    rclpy.init(args=args)

    pid = PID(
        kP_position=np.array([1.5, 1.5, 1.5]),
        kD_position=np.array([0.05, 0.05, 0.05]),
        kI_position=np.array([0.2, 0.2, 0.2]),
        kP_orientation=np.array([0.025, 0.025, 0.025]),
        kD_orientation=np.array([0.025, 0.025, 0.025]),
        kI_orientation=np.array([0, 0, 0]),
        max_integral_position=np.array([1, 1, 1]),
        max_integral_orientation=np.array([1, 1, 1]),
    )

    controller_node = Controller(pid)

    try:
        rclpy.spin(controller_node)
    except KeyboardInterrupt:
        controller_node.get_logger().info("Shutting down controller...")
        stop = WrenchStamped()
        stop.header.stamp = controller_node.get_clock().now().to_msg()
        # all fields of stop.wrench are already zero by default
        controller_node.control_publisher.publish(stop)
        pass

    controller_node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
