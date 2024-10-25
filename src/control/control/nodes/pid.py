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
    spatialmath-python
    control.utils.state.State
    control.utils.wrench.AbstractWrench
    geometry_msgs.msg.WrenchStamped

Author:
    Ali Ahmad

Version:
    1.0.0
"""

from state import State
from wrench import AbstractWrench

from geometry_msgs.msg import WrenchStamped

import numpy as np

import spatialmath as sm

from src/msgs/msg/GeneratedPath.msg import Paths 

class PID_config():
    "Class to encapsulate PID parameters"
    "Kp - proportional gain"
    "Kd - derivative gain"
    "Ki - integral gain"
    "Three sets of gains for position, velocity, and orientation"
    "Max signals : upper limits for the control signal"
    def __init__(self, 
                kP_position: np.ndarray,
                kD_position: np.ndarray,
                kI_position: np.ndarray,
                kP_velocity: np.ndarray,
                kD_velocity: np.ndarray,
                kI_velocity: np.ndarray,
                kP_orientation: np.ndarray,
                kD_orientation: np.ndarray,
                kI_orientation: np.ndarray,
                kP_angular_velocity: np.ndarray,
                kD_angular_velocity: np.ndarray,
                kI_angular_velocity: np.ndarray,
                max_signal_position: np.ndarray,
                max_signal_velocity: np.ndarray,
                max_signal_orientation: np.ndarray,
                max_signal_angular_velocity: np.ndarray,
                max_integral_position: np.ndarray,
                max_integral_velocity: np.ndarray,
                max_integral_orientation: np.ndarray,
                max_integral_angular_velocity: np.ndarray):

        self.kP_position = kP_position
        self.kD_position = kD_position
        self.kI_position = kI_position

        self.kP_velocity = kP_velocity
        self.kD_velocity = kD_velocity
        self.kI_velocity = kI_velocity

        self.kP_orientation = kP_orientation
        self.kD_orientation = kD_orientation
        self.kI_orientation = kI_orientation

        self.kP_angular_velocity = kP_angular_velocity
        self.kD_angular_velocity = kD_angular_velocity
        self.kI_angular_velocity = kI_angular_velocity

        self.integral_position = np.zeros(3)
        self.integral_velocity = np.zeros(3)
        self.integral_orientation = np.zeros(3)
        self.integral_angular_velocity = np.zeros(3)

        self.max_signal_position = max_signal_position
        self.max_signal_orientation = max_signal_orientation
        self.max_signal_velocity = max_signal_velocity
        self.max_signal_angular_velocity = max_signal_angular_velocity

        self.max_integral_position = max_integral_position
        self.max_integral_orientation = max_integral_orientation
        self.max_integral_velocity = max_integral_velocity
        self.max_integral_angular_velocity = max_integral_angular_velocity



    def update_PID_params(self, gain_type : str, new_values : np.ndarray):
        if hasattr(self, gain_type):
            setattr(self, gain_type, new_values)
        else:
            raise ValueError("invalid gain type :(")

class PID():
    """
    PID controller class that minimizes error to a desired reference.

    Computes a control signal based on the error between the system's current
    state and the desired reference point, with configurable limits and tuning
    parameters.
    """

    def __init__(self, config: PID_config):
        
        self.config = config
        self.index = 0 # index of where we are in the paths 

        self.cur_state = None
        self.reference = None
        self.paths = None



        self.lastPosError = np.zeros(3)
        self.lastVelError = np.zeros(3)
        self.lastOrientationError = np.zeros(3)
        self.lastangVelError = np.zeros(3)

    def set_cur_state(self, state: State):
        self.cur_state = state
    
    def set_reference_state(self, reference: State):
        self.reference = reference

    def calculatePosOutput(self, dt):
        #Get time change
        self.timeFunction() 

        #Calculate error' 
        posError = self.reference.position_world - self.cur_state.position_world
        #Proportional' 
        pTerm = self.config.kP_position * posError
        #Integral' 
        self.config.integral_position += posError * dt
        self.config.integral_position = np.clip(self.config.integral_position, -self.config.max_integral_position, self.config.max_integral_position)
        iTerm = self.config.kI_position * self.config.integral_position
        #Derivative' 
        dTerm = self.config.kD_position * (posError - self.lastPosError) / dt
        #Sum' 
        output = pTerm + iTerm + dTerm
        #Clamp' 
        output = np.clip(output, -self.config.max_signal_position, self.config.max_signal_position)
        #Update last error' 
        self.lastPosError = posError
        return output

    def calculateVelOutput(self, dt):
        #Get time change
        self.timeFunction() 

        #Calculate error' 
        velError = self.reference.velocity_body - self.cur_state.velocity_body
        #Proportional' 
        pTerm = self.config.kP_velocity * velError
        #Integral' 
        self.config.integral_velocity += velError * dt
        self.config.integral_velocity = np.clip(self.config.integral_velocity, -self.config.max_integral_velocity, self.config.max_integral_velocity)
        iTerm = self.config.kI_velocity * self.config.integral_velocity
        
        #Derivative' 
        dTerm = self.config.kD_velocity * (velError - self.lastVelError) / dt
        #Sum' 
        output = pTerm + iTerm + dTerm
        #Clamp' 
        output = np.clip(output, -self.config.max_signal_velocity, self.config.max_signal_velocity)
        #Update last error' 
        self.lastVelError = velError
        return output

    def calculateOrientationOutput(self, dt):
        #Get time change
        self.timeFunction()

        orientationError = self.reference.orientation_world * self.cur_state.orientation_world.inv()
        
        angle, axis = orientationError.angle_axis()
        orientationError = angle * axis


        #Proportional
        pTerm = self.config.kP_orientation * orientationError
        #Integral
        self.config.integral_orientation += orientationError * dt
        self.config.integral_orientation = np.clip(self.config.integral_orientation, -self.config.max_integral_orientation, self.config.max_integral_orientation)
        iTerm = self.config.kI_orientation * self.config.integral_orientation
        #Derivative
        dTerm = self.config.kD_orientation * (orientationError - self.lastOrientationError) / dt
        #Sum
        output = pTerm + iTerm + dTerm
        #Clamp
        output = np.clip(output, -self.config.max_signal_orientation, self.config.max_signal_orientation)
        #Update last error
        self.lastOrientationError = orientationError
        return output

    def calculateAngularVelocityOutput(self, dt):
        # Get time change
        self.timeFunction() 

        #Calculate Error
        angVelError = self.reference.angular_velocity_body - self.cur_state.angular_velocity_body
        # Proportional
        propTerm = angVelError * self.config.kP_angular_velocity
        #Derivative
        d_dx = (angVelError - self.lastangVelError) / dt 
        d_dx *= self.config.kD_angular_velocity
        #Integraive
        self.config.integral_orientation += angVelError * dt
        self.config.integral_orientation = np.clip(self.config.integral_orientation, -self.config.max_integral_angular_velocity, self.config.max_integral_angular_velocity)
        iTerm = angVelError * self.config.kI_angular_velocity * dt 
        #generate output
        output = propTerm + d_dx + iTerm 

        self.lastangVelError = angVelError

        return output
        


    def reset(self):
        """
        Reset the integral term of the PID controller.

        This function is used to reset the accumulated integral error, which
        can be useful to avoid wind-up issues in the controller.
        """
        self.config.integral_position = np.zeros(3)
        self.config.integral_orientation = np.zeros(3)
        self.config.integral_velocity = np.zeros(3)
        self.config.integral_angular_velocity = np.zeros(3)
        self.lastPosError = np.zeros(3)
        self.lastVelError = np.zeros(3)
        self.lastOrientationError = np.zeros(3)
        self.lastangVelError = np.zeros(3)

    

    def areWeThereYet(self, error_margin = 1):
        #Check if we are at the end of the path
        if self.index == len(self.paths) - 1:
            return True

        #Check if we are close enough to the next point in the path

        errorMagnitude = np.linalg.norm(self.lastPosError)
        if errorMagnitude <= error_margin:
            self.index += 1
            self.reference = State.from_customPaths_msg(self.paths, self.index)
            return True
        return False




    def update(sel, dt) -> WrenchStamped:

        if self.cur_state is None or self.reference is None:
            raise ValueError("Current state or reference state is not set")
        
        if self.paths is None:
            raise ValueError("No paths are set")
        
        update_index = self.areWeThereYet()
            
        
        force_body = self.calculatePosOutput(dt) + self.calculateVelOutput(dt)
        torque_body = self.calculateOrientationOutput(dt) + self.calculateAngularVelocityOutput(dt)
        
        wrench = AbstractWrench(force_body, torque_body)

        return wrench.to_msg()

    
