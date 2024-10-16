"""
Utility functions for the control module.

Author:
    Ali Ahmad

Version:
    1.0.1
"""
from nav_msgs.msg import Odometry

import numpy as np

import quaternion


def to_np(o: Odometry) -> np.array:
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
    raise ValueError('Unsupported type for to_np')


def quat_to_axis_angle(q: quaternion) -> np.array:
    """Convert a quaternion to an axis-angle representation."""
    q = q.normalized()
    w = q.w
    v = np.array([q.x, q.y, q.z])
    theta = 2 * np.arccos(w)
    normv = np.sqrt(1 - w**2)
    axis = np.zeros(3) if normv < 1e-10 else v / normv
    return axis * theta


def qvmul(q: quaternion, v: np.array) -> np.array:
    """Multiply a quaternion with a vector."""
    q = q.normalized()
    v = quaternion.quaternion(0, *v)
    return (q * v * q.conjugate()).imaginary
