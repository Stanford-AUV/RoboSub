"""
Controller Node.

Authors:
    Ali Ahmad
    Khaled Messai


"""

import threading

from geometry_msgs.msg import WrenchStamped

from nav_msgs.msg import Odometry

import numpy as np

import rclpy
from rclpy.node import Node


class Controller(Node):
    """
    Node that computes a control signal based on state and reference poses.

    Creates subscribers to the state and reference topics and publishes to the 
    desired_wrench topic. Uses a PID controller to compute the control signal 
    with configurable gains and limits.
    """

    def __init__(self, k_pos, k_ang, ceil):
        """
        Initialize the controller node.

        We initialize the controller node with subscribers to the state and
        reference topics, and a publisher to the desired_wrench topic. We
        initialize the PID controller with given parameters and a lock for
        thread safety.

        Parameters
        ----------
        k_pos: np.array (3x1)
            The position PID controller gains.
        k_ang: np.array (3x1)
            The angular PID controller gains.
        ceil: np.array (2x1)
            The saturation limits for the controller.
        start_i: np.array (2x1)
            The initial integral error for the controller.
        """
        super().__init__('controller')

        self.state_subscription = self.create_subscription(
            Odometry, 'state', self.state_callback, 10
        )
        self.reference_subscription = self.create_subscription(
            Odometry, 'reference', self.reference_callback, 10
        )
        self.control_publisher = self.create_publisher(
            WrenchStamped, 'desired_wrench', 10
        )

        self.lock = threading.Lock()

        self.time = self.get_clock().now()

        self.k_pos = np.diag(k_pos)
        self.k_ang = np.diag(k_ang)
        self.max = ceil


    def state_callback(self):
        pass

    def reference_callback(self):
        pass

    def update(self):
        pass

    def reset(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
