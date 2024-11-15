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
    Khaled Messai

Version:
    1.0.0
"""

from state import State
from wrench import AbstractWrench

from geometry_msgs.msg import WrenchStamped

import numpy as np

import spatialmath as sm

from src/msgs/msg/GeneratedPath.msg import Paths 

from pid_config import PID_config
    
        

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
        self.paths = None #keep track of the paths


        #derivative term requires last errors
        self.lastPosError = np.zeros(3)
        self.lastVelError = np.zeros(3)
        self.lastOrientationError = np.zeros(3)
        self.lastangVelError = np.zeros(3)

    def set_cur_state(self, state: State):
        self.cur_state = state
    
    def set_reference_state(self, reference: State):
        self.reference = reference

    def set_paths(self, paths: Paths):
        self.paths = paths

    def reset_path_index(self):
        self.index = 0
        self.paths = None


    'The PID calculations for each of the four possible errors. Position, Velocity, Orientation, Angular Velocity.'
    'These functions are presently kept separate to prepare for ad hoc modifications to their calculations. It is possible to combine them into one function,' 
    'with the exception of orientation, which calculates the error differently.'

    'Core PID loop:'
        '1. Calculate error, represented as the difference between the reference and the current state.'
        '2. Calculate the proportional term, which is the error multiplied by the proportional gain (kp).'
        '3. Calculate the integral term, which is the sum of the error multiplied by the integral gain (ki).'
        '4. Calculate the derivative term, which is the difference between the current error and the previous error multiplied by the derivative gain (kd).'
        '5. Sum the three terms to get the output.'

    def calculatePosOutput(self, dt):
        #Calculate error' 
        posError = self.reference.position_world - self.cur_state.position_world
        #Proportional' 
        pTerm = self.config.kP_position * posError
        #Integral' 
        self.config.integral_position += posError * self.config.kI_position * dt
        iTerm = self.config.integral_position 
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
        #Calculate error' 
        velError = self.reference.velocity_body - self.cur_state.velocity_body
        #Proportional' 
        pTerm = self.config.kP_velocity * velError
        #Integral' 
        self.config.integral_velocity += velError * self.config.kI_velocity * dt
        iTerm = self.config.integral_velocity
        
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
        orientationError = self.reference.orientation_world * self.cur_state.orientation_world.inv()
        
        angle, axis = orientationError.angle_axis()
        orientationError = angle * axis


        #Proportional
        pTerm = self.config.kP_orientation * orientationError
        #Integral
        self.config.integral_orientation += orientationError * self.config.kI_orientation * dt
        iTerm = self.config.integral_orientation
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
        #Calculate Error
        angVelError = self.reference.angular_velocity_body - self.cur_state.angular_velocity_body
        # Proportional
        propTerm = angVelError * self.config.kP_angular_velocity
        #Derivative
        dTerm = self.config.kD_angular_velocity * (angVelError - self.lastangVelError) / dt 
        #Integrative
        self.config.integral_angular_velocity += angVelError * self.config.kI_angular_velocity * dt
        iTerm = self.config.integral_angular_velocity 
        #generate output
        output = propTerm + dTerm + iTerm 

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
        #Check if we are close enough to the next point in the path
        errorMagnitude = np.linalg.norm(self.lastPosError)
        if errorMagnitude <= error_margin:
            if self.index < len(self.paths) - 1:
                self.index += 1
                self.reference = State.from_customPaths_msg(self.paths, self.index)
            return True
        return False




    def update(self, dt) -> WrenchStamped:
        
        if self.paths is None:
            self.get_logger().warn("No paths set; skipping update.")
            return None
        elif self.cur_state is None:
            self.get_logger().warn("No current state set; skipping update.")
            return None
        elif self.reference is None:
            self.get_logger().warn("No reference state set; skipping update.")
            return None
        elif not np.isfinite(dt) or dt <= 0:
            self.get_logger().warn("Invalid dt value; skipping update.")
            return None
        else:
            update_index = self.areWeThereYet()
            force_body = self.calculatePosOutput(dt) + self.calculateVelOutput(dt)
            torque_body = self.calculateOrientationOutput(dt) + self.calculateAngularVelocityOutput(dt)
            wrench = AbstractWrench(force_body, torque_body)
            return wrench.to_msg()

            
        
        
        
        

        

    
