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
        super().__init__('controller')

        self.state_subscription = self.create_subscription(
            Odometry, 'odometry', self.state_callback, 10
        )
        self.reference_subscription = self.create_subscription(
            Paths, 'path', self.reference_callback, 10
        )

        self.reset_path_subscription = self.create_subscription(
            Empty, 'reset_path', self.reset_path_index, 10
        )
        self.control_publisher = self.create_publisher(
            WrenchStamped, 'wrench', 10
        )



        self.lock = threading.Lock()

        self.time = self.get_clock().now()

        self.policy = policy

        self.cur_state = None
        self.ref_state = None

    def reset(self):
        """Reset the controller."""
        self.get_logger.info('Resetting controller...')
        with self.lock:
            self.policy.reset()

    def reset_path_index(self, msg):
        self.get_logger().info('Resetting path index')
        self.policy.reset_path_index()

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
            self.policy.set_cur_state(self.cur_state)
            self.get_logger().info('Current state updated')
            self.update()

    def reference_callback(self, msg: Odometry):
        """
        Update the reference state.

        Parameters
        ----------
        msg : Odometry
            The message containing the reference state.
        """
        with self.lock:
            self.ref_state = State.from_customPaths_msg(msg)
            self.policy.set_reference(self.ref_state)
            self.policy.paths = msg.paths
            self.get_logger().info('Reference state updated')


    def deltaT(self):
        newTime = self.get_clock().now()
        dt = (newTime - self.time) / 1e9
        self.time = newTime
        return dt

    def update(self):
        """Update the control signal and publish to the wrench topic."""
        dt = self.deltaT()
        try:
            wrench = self.policy.update(self.cur_state, self.ref_state, dt)
            self.control_publisher.publish(wrench)
        except Exception as e:
            self.get_logger().error(f'Error updating control signal: {e}')
            self.get_logger().error('Resetting controller...')
            self.reset()



def main(args=None):
    """Set up and spin the Controller node."""
    rclpy.init(args=args)

    pid = PID(
        kP_position=np.array([1, 1, 1]),
        kD_position=np.array([1, 1, 1]),
        kI_position=np.array([1, 1, 1]),
        kP_orientation=np.array([1, 1, 1]),
        kD_orientation=np.array([1, 1, 1]),
        kI_orientation=np.array([1, 1, 1]),
        max_signal_position=np.array([1, 1, 1]),
        max_signal_orientation=np.array([1, 1, 1]),
        max_integral_position=np.array([1, 1, 1]),
        max_integral_orientation=np.array([1, 1, 1])
    )

    controller_node = Controller(pid)

    try:
        rclpy.spin(controller_node)
    except KeyboardInterrupt:
        controller_node.get_logger().info('Shutting down controller...')
        pass

    controller_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
