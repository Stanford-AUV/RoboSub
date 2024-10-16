"""
PID Controller Module.

This modules contains a simple implementation for a PID controller. The PID
controller is a feedback controller that outputs a control signal to minimize
the error between the current state and a reference state. The control signal
is tunable through the kp, ki, and kd gains.

This PID minimizes error for the position and velocity in one axis only. The
intent of the controller design is to apply a signal

Classes:
    PID: Implements a PID controller with configurable gains and limits.

Usage:
    To use this module, create an instance of the PID class with the desired
    gains and limits, and use the `update()` method to compute control outputs.

Example:
    pid = PID([1, 1, 1], [1, 1, 1], [1, 1, 1], 10, [1, 1, 1], [1, 1, 1])
    control_signal = pid.update(state, reference, dt)

Dependencies:
    numpy
    quaternion
    control.utils.state.State
    control.utils.wrench.AbstractWrench
    geometry_msgs.msg.WrenchStamped

Author:
    Ali Ahmad

Version:
    1.0.0
"""

from control.utils.state import State
from control.utils.utils import quat_to_axis_angle
from control.utils.wrench import AbstractWrench

from geometry_msgs.msg import WrenchStamped

import numpy as np

import quaternion


class PID():
    """
    PID controller class that minimizes error to a desired reference.

    Computes a control signal based on the error between the system's current
    state and the desired reference point, with configurable limits and tuning
    parameters.
    """

    def __init__(self,
                 kP_position: np.array,
                 kD_position: np.array,
                 kI_position: np.array,
                 kP_orientation: np.array,
                 kD_orientation: np.array,
                 kI_orientation: np.array,
                 max_signal_position: np.array,
                 max_signal_orientation: np.array,
                 max_integral_position: np.array,
                 max_integral_orientation: np.array):
        """
        Initialize the PID controller gains and limits.

        Parameters
        ----------
        kP_position : np.array
            The proportional gains for the position error.
        kD_position : np.array
            The derivative gains for the position error.
        kI_position : np.array
            The integral gains for the position error.
        kP_orientation : np.array
            The proportional gains for the orientation error.
        kD_orientation : np.array
            The derivative gains for the orientation error.
        kI_orientation : np.array
            The integral gains for the orientation error.
        max_signal_position : np.array
            The maximum control signal for the position.
        max_signal_orientation : np.array
            The maximum control signal for the orientation.
        max_integral_position : np.array
            The maximum integral error for the position.
        max_integral_orientation : np.array
            The maximum integral error for the orientation.
        """
        self.kP_position = kP_position
        self.kD_position = kD_position
        self.kI_position = kI_position
        self.kP_orientation = kP_orientation
        self.kD_orientation = kD_orientation
        self.kI_orientation = kI_orientation

        self.integral_position = np.array([0, 0, 0])
        self.integral_orientation = np.array([0, 0, 0])
        self.max_signal_position = max_signal_position
        self.max_signal_orientation = max_signal_orientation
        self.max_integral_position = max_integral_position
        self.max_integral_orientation = max_integral_orientation

    def reset(self):
        """
        Reset the integral term of the PID controller.

        This function is used to reset the accumulated integral error, which
        can be useful to avoid wind-up issues in the controller.
        """
        for key in self.iSum:
            self.iSum[key] = 0

    def update(self,
               state: State,
               reference: State,
               dt: float
               ) -> WrenchStamped:
        """
        Compute the control signal.

        The function calculates the control signal by evaluating the between
        the error between the current state and the target reference. It then
        computes the control signal by multiplying the error by the gains
        and clamp the output to the maximum limit.

        Parameters
        ----------
        state : State
            State object representing the current state of the system.
        reference : State
            State object representing the desired reference state.
        dt : float
            The time step between the current and previous state.

        Returns
        -------
        np.array
            The control signal, constrained by the maximum output limit (ceil).
        """
        # Position error, world frame
        error_r_W = reference.position_world - state.position_world
        # Velocity error, body frame
        error_v_B = reference.velocity_body - state.velocity_body
        # Integral sum, clamped
        self.integral_position += error_r_W * dt
        self.integral_position = np.clip(
            self.integral_position,
            -self.max_integral_position,
            self.max_integral_position
        )
        # Calculate force in the world frame
        force_world = error_r_W * self.kP_position + \
            error_v_B * self.kD_position + \
            self.integral_position * self.kI_position

        # Orientation error, world frame, axis-angle form
        error_q_W = reference.orientation_world * \
            state.orientation_world.inverse()
        error_q_W = quat_to_axis_angle(error_q_W)
        # Angular velocity error, body frame
        error_w_B = reference.angular_velocity_body - \
            state.angular_velocity_body
        self.integral_orientation += error_q_W * dt
        self.integral_orientation = np.clip(
            self.integral_orientation,
            -self.max_integral_orientation,
            self.max_integral_orientation
        )

        # Calculate torque in the body frame
        torque_body = error_q_W * self.kP_orientation + \
            error_w_B * self.kD_orientation + \
            self.integral_orientation * self.kI_orientation

        # Convert force to body frame
        force_quaternion = quaternion.quaternion(0, *force_world)
        force_body = state.orientation_world.inverse() *\
            force_quaternion * state.orientation_world

        # Clamp the control signal
        force_body = np.clip(
            force_body,
            -self.max_signal_position,
            self.max_signal_position
        )
        torque_body = np.clip(
            torque_body,
            -self.max_signal_orientation,
            self.max_signal_orientation
        )

        wrench = AbstractWrench(force_body, torque_body)

        return wrench.to_msg()
