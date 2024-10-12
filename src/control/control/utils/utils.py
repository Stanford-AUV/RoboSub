"""
Utility functions for the control module.

Author:
    Ali Ahmad

Version:
    1.0.0
"""
from nav_msgs.msg import Odometry

import numpy as np


def to_np(o: Odometry):
    """Convert a ROS odometry message to a numpy array."""
    if isinstance(o, Odometry):
        return np.array([
            o.pose.pose.position.x,
            o.pose.pose.position.y,
            o.pose.pose.position.z,
            o.twist.twist.linear.x,
            o.twist.twist.linear.y,
            o.twist.twist.linear.z,
            o.pose.pose.orientation.x,
            o.pose.pose.orientation.y,
            o.pose.pose.orientation.z,
            o.pose.pose.orientation.w,
            o.twist.twist.angular.x,
            o.twist.twist.angular.y,
            o.twist.twist.angular.z
        ])
    raise ValueError("Unsupported type for toNp")