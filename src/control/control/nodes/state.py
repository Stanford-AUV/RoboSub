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

Dependencies:
    numpy
    nav_msgs.msg.Odometry

Author:
    Ali Ahmad
    Khaled Messai

Version:
    1.0.0
"""
from nav_msgs.msg import Odometry

import numpy as np

import spatialmath as sm


class State():
    """
    Hold the values representing the state of the system.

    Represents the state of a robot in 3D space, including its position,
    velocity, orientation, and angular velocity. This class is used to
    manage and manipulate the state information effectively.
    """

    def __init__(self,
                 position_world: np.ndarray,
                 velocity_body: np.ndarray,
                 orientation_world: sm.SE3,
                 angular_velocity_body: np.ndarray):
        """
        Initialize a State object.

        Parameters
        ----------
        position_world : np.ndarray
            A 3-vector representing the position of the robot in the world.
        velocity_body : np.ndarray
            A 3-vector representing the velocity of the robot in the body
            frame.
        orientation_world : np.ndarray
            A 4-vector representing the orientation of the robot in the world.
        angular_velocity_body : np.ndarray
            A 3-vector representing the angular velocity of the robot in the
            body frame.
        """
        self.position_world = position_world
        self.velocity_body = velocity_body
        self.orientation_world = orientation_world
        self.angular_velocity_body = angular_velocity_body

    @staticmethod
    def from_odometry_msg(msg: Odometry):
        """
        Convert an Odometry message to a State object.

        Parameters
        ----------
        msg : Odometry
            Odometry message containing the state information.
        """
        position_world = np.array(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ]
        )
        velocity_body = np.array(
            [
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
            ]
        )
        orientation_world = sm.SE3.Quaternion(
            s=msg.pose.pose.orientation.w,
            v=[
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
            ]
        )
        angular_velocity_body = np.array(
            [
                msg.twist.twist.angular.x,
                msg.twist.twist.angular.y,
                msg.twist.twist.angular.z,
            ]
        )

        return State(
            position_world,
            velocity_body,
            orientation_world,
            angular_velocity_body
        )


    @staticmethod
    def from_customPaths_msg(msg: Paths, index=0: int):
        #Path Message:
        #List of Poses 
        #List of Twists 

        #Given an index, grab the path and twist at that index in the list, and return a state. 

         """
        Parameters
        ----------
        msg : Paths
            Custom stamped Robosub paths message containing a:
                1. List of Pose messages
                2. List of Twist messages
        index : int
            The index of the path and twist we want to convert to the reference state. 
            Default: 0
        """
        position_world = np.array(
            [
                msg.poses[index].pose.position.x,
                msg.poses[index].pose.position.y,
                msg.poses[index].pose.position.z,
            ]
        )
        velocity_body = np.array(
            [
                msg.twists[index].twist.linear.x,
                msg.twists[index].twist.linear.y,
                msg.twists[index].twist.linear.z,
            ]
        )
        orientation_world = sm.SE3.Quaternion(
            s=msg.poses[index].pose.orientation.w,
            v=[
                msg.poses[index].pose.orientation.x,
                msg.poses[index].pose.orientation.y,
                msg.poses[index].pose.orientation.z,
            ]
        )
        angular_velocity_body = np.array(
            [
                msg.twists[index].twist.angular.x,
                msg.twists[index].twist.angular.y,
                msg.twists[index].twist.angular.z,
            ]
        )

        return State(
            position_world,
            velocity_body,
            orientation_world,
            angular_velocity_body
        )

        



