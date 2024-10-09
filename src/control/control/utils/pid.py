"""
PID Controller Module.

This modules contains a simple implementation for a PID controller. The PID
controller is a feedback controller that outputs a control signal to minimize
the error between the current state and a reference state. The control signal
is tunable through the kp, ki, and kd gains.

Classes:
    PID: Implements a PID controller with configurable gains and limits.

Usage:
    To use this module, create an instance of the PID class with the desired
    gains and limits, and use the `output()` method to compute control outputs.

Example:
    pid = PID(1.0, 0.5, 0.2, 100, 5.0)
    signal = pid.output(current_state, reference_state)

Dependencies:
    numpy

Author:
    Ali Ahmad

Version:
    1.0.0
"""
import numpy as np


class PID():
    """
    PID controller class that minimizes error to a desired reference.

    Computes a control signal based on the error between the system's current
    state and the desired reference point, with configurable limits and tuning
    parameters.
    """

    def __init__(self,
                 kp: float,
                 ki: float,
                 kd: float,
                 ceil: float, 
                 start_i: float):
        """
        Initialize the PID controller with gains and limits.

        Parameters
        ----------
        kp : float
            Proportional gain for the PID controller. Controls the reaction to
            the current error.
        ki : float
            Integral gain for the PID controller. Accumulates past errors to
            eliminate steady-state errors.
        kd : float
            Derivative gain for the PID controller. Predicts future error by
            observing the rate of change.
        ceil : float
            Maximum output limit for the control signal (e.g., max torque/
            force).
        start_i : float
            The initial value for the integral term (typically starts at 0, but can be configured).
        """
        self.gains = np.diag([kp, ki, kd])
        self.ceil = ceil
        self.start_i = start_i
        self.integral = 0

    def output(self, state: np.array, reference: np.array) -> float:
        """
        Compute the control signal based on the current state and the 
        desired reference. 

        The function calculates the control signal by evaluating the

        Parameters
        ----------
        state : _type_
            _description_
        reference : _type_
            _description_
        """
        error = reference - state

    def reset(self):
        """
        Reset the integral term of the PID controller.

        This function is used to reset the accumulated integral error, which can be useful
        to avoid wind-up issues in the controller.
        """
        self.integral = 0
