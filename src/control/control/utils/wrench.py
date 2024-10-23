"""
Wrench object to abstract conversion from numpy arrays to ROS messages.

Usage:
    To use this module, create an instance of the Wrench class with the desired
    force and torque vectors, and call the `to_msg()` method to convert the
    object to a ROS WrenchStamped message.

Example:
    wrench = Wrench(np.array([1, 2, 3]), np.array([4, 5, 6]))
    msg = wrench.to_msg()

Dependencies:
    geometry_msgs.msg.Vector3
    geometry_msgs.msg.Wrench
    geometry_msgs.msg.WrenchStamped
    numpy
    rclpy.time.Time
    
Author:
    Ali Ahmad

Version:
    1.0.0
"""
from geometry_msgs.msg import Vector3, Wrench, WrenchStamped

import numpy as np

from rclpy.time import Time


class AbstractWrench():
    """Wrench class to abstract conversion of numpy arrays to ROS messages."""

    def __init__(self, force: np.array, torque: np.array):
        """
        Initialize a Wrench object with force and torque vectors.

        Parameters
        ----------
        force : np.array
            The force vector.
        torque : np.array
            The torque vector.
        """
        self.force = force
        self.torque = torque
        self.wrench_msg = Wrench(Vector3(force), Vector3(torque))

    def to_msg(self):
        """Convert the Wrench object to a ROS WrenchStamped message."""
        msg = WrenchStamped()
        msg.header.stamp = Time().now().to_msg()
        msg.wrench = self.wrench_msg
        return msg
