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

from nav_msgs.msg import Odometry

import numpy as np

import rclpy
from rclpy.node import Node


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

        self.cur_state = None
        self.ref_state = None

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
        with self.lock:
            self.cur_state = State.from_odometry_msg(msg)
            self.update() if self.ref_state is not None else None

    def reference_callback(self, msg: Odometry):
        """
        Update the reference state.

        Parameters
        ----------
        msg : Odometry
            The message containing the reference state.
        """
        with self.lock:
            self.ref_state = State.from_odometry_msg(msg)
            self.get_logger().info("Reference state updated")

    def update(self):
        """Update the control signal and publish to the wrench topic."""
        newTime = self.get_clock().now()
        dt = (newTime - self.time).nanoseconds / 1e9
        self.time = newTime
        wrench = self.policy.update(self.cur_state, self.ref_state, dt)
        wrench.header.stamp = self.time.to_msg()
        self.control_publisher.publish(wrench)


def main(args=None):
    """Set up and spin the Controller node."""
    rclpy.init(args=args)

    pid = PID(
        kP_position=np.array([10, 10, 10]),
        kD_position=np.array([11, 11, 11]),
        kI_position=np.array([0, 0, 0]),
        kP_orientation=np.array([0.5, 0.1, 1.0]),
        kD_orientation=np.array([2.0, 0.2, 2.0]),
        kI_orientation=np.array([0, 0, 0]),
        max_signal_force=np.array([0.2, 0.2, 0.2]),
        max_signal_torque=np.array([0.4, 0.4, 0.01]),
        max_integral_position=np.array([1, 1, 1]),
        max_integral_orientation=np.array([1, 1, 1]),
    )

    controller_node = Controller(pid)

    try:
        rclpy.spin(controller_node)
    except KeyboardInterrupt:
        controller_node.get_logger().info("Shutting down controller...")
        pass

    controller_node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
