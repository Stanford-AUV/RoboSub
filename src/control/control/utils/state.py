"""
State class to represent the state of a robot in 3D space.

This module defines the State class, which represents the state of a robot
in terms of position, velocity, orientation, and angular velocity. It also
provides a utility method to convert a ROS Odometry message into a State
object.

Classes:
    State: Represents the state of a robot in 3D space.

Usage:
    To use this module, create an instance of the State class with the desired
    state information, or use the `from_odometry_msg()` method to convert a
    ROS Odometry message to a State object.

Example:
    state = State(
        np.array([0, 0, 0]),
        np.array([0, 0, 0]),
        np.array([1, 0, 0, 0]),
        np.array([0, 0, 0])
    )
    state = State.from_odometry_msg(odom_msg)
    state =  State.from_generatedpath_msg(path_msg, index)

Dependencies:
    numpy
    nav_msgs.msg.Odometry

Author:
    Ali Ahmad
    Khaled Messai

Version:
    1.0.0
"""

from msgs.msg import GeneratedPath

from nav_msgs.msg import Odometry

import numpy as np

import spatialmath as sm

from dataclasses import dataclass

@dataclass
class Magnitude:
    distance: float
    speed: float
    angle: float
    angular_speed: float

    def __gt__(self, other: 'Magnitude'):
        return self.distance > other.distance and self.speed > other.speed and self.angle > other.angle and self.angular_speed > other.angular_speed

class State:
    """
    Hold the values representing the state of the system.

    Represents the state of a robot in 3D space, including its position,
    velocity, orientation, and angular velocity. This class is used to
    manage and manipulate the state information effectively.
    """

    def __init__(self,
                 position: np.ndarray,
                 velocity: np.ndarray,
                 orientation: sm.SE3,
                 angular_velocity: np.ndarray):
        """
        Initialize a State object.

        Parameters
        ----------
        position : np.ndarray
            A 3-vector representing the position of the robot in the world frame.
        velocity : np.ndarray
            A 3-vector representing the velocity of the robot in the world frame.
        orientation : np.ndarray
            A 4-vector representing the orientation of the robot in the world frame.
        angular_velocity : np.ndarray
            A 3-vector representing the angular velocity of the robot in the world frame.
        """
        self.position = position
        self.velocity = velocity
        self.orientation = orientation
        self.angular_velocity = angular_velocity

    def __sub__(self, other: 'State'):
        return State(
            self.position - other.position,
            self.velocity - other.velocity,
            self.orientation * other.orientation.inv(),
            self.angular_velocity - other.angular_velocity
        )
    
    def magnitude(self):
        position_magnitude = np.linalg.norm(self.position)
        velocity_magnitude = np.linalg.norm(self.velocity)
        angle, _ = self.orientation.angvec()
        angle_magnitude = np.linalg.norm(angle)
        angular_velocity_magnitude = np.linalg.norm(self.angular_velocity)
        return Magnitude(
            distance=position_magnitude,
            speed=velocity_magnitude,
            angle=angle_magnitude,
            angular_speed=angular_velocity_magnitude
        )
    
    @staticmethod
    def from_odometry_msg(msg: Odometry):
        """
        Convert an Odometry message to a State object.

        Parameters
        ----------
        msg : Odometry
            Odometry message containing the state information.
        """
        position = np.array(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ]
        )
        velocity = np.array(
            [
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
            ]
        )
        orientation = sm.UnitQuaternion(
            s=msg.pose.pose.orientation.w,
            v=[
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
            ]
        )
        angular_velocity = np.array(
            [
                msg.twist.twist.angular.x,
                msg.twist.twist.angular.y,
                msg.twist.twist.angular.z,
            ]
        )

        return State(
            position,
            velocity,
            orientation,
            angular_velocity
        )

    @staticmethod
    def from_generatedpath_msg(msg: GeneratedPath, index=0):
        """
        Convert a GeneratedPath message with an index to a State object.

        Parameters
        ----------
        msg : Paths
            Custom stamped Robosub paths message containing a:
                1. List of Pose messages
                2. List of Twist messages
        index : int
            The index of the path and twist we want to convert to the
            reference state.
            Default: 0
        """
        position = np.array(
            [
                msg.poses[index].pose.position.x,
                msg.poses[index].pose.position.y,
                msg.poses[index].pose.position.z,
            ]
        )
        velocity = np.array(
            [
                msg.twists[index].twist.linear.x,
                msg.twists[index].twist.linear.y,
                msg.twists[index].twist.linear.z,
            ]
        )
        orientation = sm.SE3.Quaternion(
            s=msg.poses[index].pose.orientation.w,
            v=[
                msg.poses[index].pose.orientation.x,
                msg.poses[index].pose.orientation.y,
                msg.poses[index].pose.orientation.z,
            ]
        )
        angular_velocity = np.array(
            [
                msg.twists[index].twist.angular.x,
                msg.twists[index].twist.angular.y,
                msg.twists[index].twist.angular.z,
            ]
        )

        return State(
            position,
            velocity,
            orientation,
            angular_velocity
        )

    def copy(self):
        return State(
            self.position.copy(),
            self.velocity.copy(),
            self.orientation.copy(),
            self.angular_velocity.copy()
        )
