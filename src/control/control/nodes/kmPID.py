import numpy as np
from typing import List, Dict 
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import WrenchStamped


class PID(Node):

    def __init__(self, kP : Dict[str, float], kI : Dict[str, float], kD : Dict[str, float], 
                iSum = Dict[str, float], maxOutput, minOutput):

        'ROS2 node initialization, thread initialization, '
        super().__init__('controller')
        self.lock = threading.Lock()
        self.state_subscription = self.create_subscription(State, "state", self.state_callback, 10)
        self.reference_subscription = self.create_subscription()
        self.control_publisher = self.create_publisher(WrenchStamped, "desired_wrench", 10)


        'PID Constants and integral Sum'
        self.kP = kP
        self.kI = ki 
        self.kD = kD 
        self.iSum = iSum  
        self.deltaT = 1 #change  
        self.curtime = self.get_clock().now()

        self.arrayLength = len(self.kP) 


        
        'PID Limits
        self.maxOutput = maxOutput
        self.minOutput = minOutput 

        'Errors Pos 
        self.curPosError = [] 
        self.lastPosError = 0

        'Errors Vel 
        self.curVelError = [] 
        self.lastVelError = 0 

        'Errors Wrench' 
        self.curWrenchError = [] 
        self.lastWrenchError = 0 

        
       'Cur Status'
        self.curVel = np.zeros(self.arrayLength)
        self.curPos = np.zeros(self.arrayLength)
        self.curWrench = np.zeros(self.arrayLength) 


    'These three functions violate DRY but I decided I would hedge that in the future, we will want'
    'specific changes to each of them. Consequently I decided to keep them separate.'
    def calculateVelOutput(self, reference):
        'Get time change
        self.timeFunction(self) 

        kP = self.kP["vel"] 
        kI = self.kI["vel"]
        kD = self.kD["vel"]
        iSum = self.iSum["vel"]

        'Calculate error' 
        velError = reference.targetVel - self.curVel
        'Proportional' 
        propTerm = velError * kP 
        'Derivative' 
        d_dx = (curVel - self.lastVelError) / self.deltaT 
        d_dx *= kD

        'Integrative' 
        iTerm = velError * kI * self.deltaT
        self.iSum += iTerm
        'Optional call to clamping if we end up needing it.'
        output = propTerm + d_dx + iTerm 

        self.lastVelError = velError

    def calculatePosOutput(self, reference):
        'Get time change
        self.timeFunction(self) 

        kP = self.kP["Pos"] 
        kI = self.kI["Pos"]
        kD = self.kD["Pos"] 
        iSum = self.iSum["Pos"]
        
        'Calculate Error'
        posError = reference.targetPos - self.curPos
        'Proportional'
        propTerm = posError * kP 
        'Derivative'
        d_dx = (self.curPos - self.lastPosError) / self.deltaT 
        d_dx *= kD 
        'Integrative' 
        iTerm = posError * kI * self.deltaT 
        self.iSum += iTerm 
        self.lastPosError = posError 

    def calculateWrenchOutput(self, reference):
        'Get time change
        self.timeFunction(self) 

        kP = self.kP["wrench"] 
        kI = self.kI["wrench"]
        kD = self.kD["wrench"] 
        iSum = self.iSum["wrench"]
        
        'Calculate Error'
        wrenchError = reference.targetWrench - self.curWrench
        'Proportional'
        propTerm = wrenchError * kP 
        'Derivative'
        d_dx = (self.curWrench - self.lastWrenchError) / self.deltaT 
        d_dx *= kD 
        'Integrative' 
        iTerm = wrenchError * kI * self.deltaT 
        iSum += iTerm 
        self.lastWrenchError = wrenchError


    def pidClamp(self, iTerm, output, maxOutput, minOutput):

        if output < maxOutput or output > minOutput: 
            self.iSum += iTerm
        elif output >= maxOutput and self.iSum < 0:
            self.iSum += iTerm 
        elif output <= minOutput and self.iSum > 0: 
            self.iSum += iTerm 

    def timeFunction(self):
        #is in nanoseconds, probably need to scale it up to seconds. just divide by 1e9 xd 
        newTime = self.get_clock().now()
        self.deltaT = newTime - self.curTime 
        self.curTime = newTime 

    def rateLimiter(self):
        pass 


def main():
    rclpy.init(args=args)
    rclpy.shutdown()
