"""
Controller Node.

Dependencies:
    geometry_msgs.msg.WrenchStamped
    nav_msgs.msg.Odometry
    numpy
    quaternion
    rclpy
    rclpy.node.Node
    rclpy.time.Time
    threading

Authors:
    Ali Ahmad
    Khaled Messai

Version:
    1.0.0
"""

import threading

#from control.utils.utils import quat_to_axis_angle, qvmul, to_np

from geometry_msgs.msg import WrenchStamped, Wrench, Vector3

from nav_msgs.msg import Odometry

from typing import List, Dict 

import numpy as np

import quaternion

import rclpy
from rclpy.node import Node
from rclpy.time import Time


class Controller(Node):
    """
    Node that computes a control signal based on state and reference poses.

    Creates subscribers to the state and reference topics and publishes to the
    desired_wrench topic. Uses a PID controller to compute the control signal
    with configurable gains and limits.
    """

    def __init__(self,
                 kP: Dict[str, float],
                 kD: Dict[str, float],
                 kI: Dict[str, float],
                 iSum: Dict[str, float],
                 max_signal,
                 max_i):
        """
        Initialize the controller node.

        We initialize the controller node with subscribers to the state and
        reference topics, and a publisher to the desired_wrench topic. We
        initialize the PID controller with given parameters and a lock for
        thread safety.

        Parameters
        ----------
        kp: Dict[str, float]
            The proportional gains for the controller (pos, ang).
        kd: Dict[str, float]
            The derivative gains for the controller (pos, ang).
        ki: Dict[str, float]
            The integral gains for the controller (pos, ang).
        iSum: Dict[str, float]
            The rolling integral sum for the controller (pos, ang).
        max_signal: np.ndarray (2x3)
            Maximum possible output.
        max_i: np.ndarray (2x3)
            Maximum possible integral sum. 
        """
        super().__init__('controller')

        self.state_subscription = self.create_subscription(
            Odometry, 'state', self.state_callback, 10
        )
        self.reference_subscription = self.create_subscription(
            Odometry, 'reference', self.reference_callback, 10
        )
        self.control_publisher = self.create_publisher(
            WrenchStamped, 'desired_wrench', 10
        )

        self.lock = threading.Lock()

        self.curTime = self.get_clock().now()

        self.kP = kP 
        self.kD = kD
        self.kI = kI
        self.iSum = iSum 

        self.max_signal = max_signal
        self.max_i = max_i

        self.state = None
        self.reference = None



    def reset(self):
        self.get_logger.info('Resetting controller...')
        with self.lock:
            for key in self.iSum:
                self.iSum[key] = 0

    def state_callback(self, msg: Odometry):
    #Callback function to process the state message from the 'state' topic.
        with self.lock:
            # Convert the Odometry message into a State object and update cur_state
            self.cur_state = State.from_odometry_msg(msg)
            self.get_logger().info('Current state updated.')

    def reference_callback(self, msg: Odometry):
    # Callback function to process the reference message from the 'reference' topic.
        with self.lock:
            # Convert the Odometry message into a State object and update ref_state
            self.ref_state = State.from_odometry_msg(msg)
            self.get_logger().info('Reference state updated.')


    # These three functions violate DRY but I decided I would hedge that in the future, we will want'
    # specific changes to each of them. Consequently I decided to keep them separate.'
    def calculateVelOutput(self):
        #Get time change
        self.timeFunction(self) 

        kP = self.kP["vel"] 
        kI = self.kI["vel"]
        kD = self.kD["vel"]
        iSum = self.iSum["vel"]

        #Calculate error' 
        velError = self.reference.velocity_body - self.cur_state.velocity_body
        #Proportional' 
        propTerm = velError * kP 
        #Derivative' 
        d_dx = (velError - self.lastVelError) / self.deltaT 
        d_dx *= kD

        #Integrative' 
        iTerm = velError * kI * self.deltaT
        iSum += iTerm
        self.iSum["vel"] = iSum #coalesce later, just useful for testing. 

        #Optional call to clamping if we end up needing it.'
        pid_output = propTerm + d_dx + iTerm 

        force_output = qvmul(self.cur_state.orientation_world, pid_output)

        self.lastVelError = velError

        return force_output 

    def calculatePosOutput(self):
        #Get time change
        self.timeFunction(self) 

        kP = self.kP["Pos"] 
        kI = self.kI["Pos"]
        kD = self.kD["Pos"] 
        iSum = self.iSum["Pos"]
        
        #Calculate Error'
        posError = self.reference.position_world - self.cur_state.position_world
        #Proportional'
        propTerm = posError * kP 
        #Derivative'
        d_dx = (posError - self.lastPosError) / self.deltaT 
        d_dx *= kD 
        #Integrative' 
        iTerm = posError * kI * self.deltaT 
        self.iSum += iTerm 
        self.lastPosError = posError 

    def calculateWrenchOutput(self):
        # Get time change
        self.timeFunction(self) 

        kP = self.kP["wrench"] 
        kI = self.kI["wrench"]
        kD = self.kD["wrench"] 
        iSum = self.iSum["wrench"]
        
        #Calculate Error
        wrenchError = self.reference.angular_velocity_body - self.cur_state.angular_velocity_body
        # Proportional
        propTerm = wrenchError * kP 
        #Derivative
        d_dx = (wrenchError - self.lastWrenchError) / self.deltaT 
        d_dx *= kD 
        #Integrative
        iTerm = wrenchError * kI * self.deltaT 
        iSum += iTerm 

        #generate output
        torque_output = propTerm + d_dx + iTerm 
        torque_world = qvmul(self.cur_state.orientation_world, torque_output)

        self.lastWrenchError = wrenchError

        return torque_world

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

    def calculateControlOutput(self):
        wrench_msg = WrenchStamped()
        linear_force = self.calculateVelOutput() 
        angular_torque = self.calculateWrenchOutput()



        wrench_msg.force = Vector3(x=linear_force[0], y=linear_force[1], z=linear_force[2])
        wrench_msg.torque = Vector3(x=angular_torque[0], y=angular_torque[1], z=angular_torque[2])
        wrench_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        wrench_stamped_msg.header.frame_id = "base_link" 

        return wrench_stamped_msg

    def publish_control(self):
        wrench_stamped_msg = self.calculateControlOutput()
        self.control_publisher.publish(wrench_stamped_msg)


        

    @staticmethod
    def qvmul(q, v):
        v_quat = np.quaternion(0, v[0], v[1], v[2])
        rotated_v = q * v_quat * np.conjugate(q)
        return np.array([rotated_v.x, rotated_v.y, rotated_v.z])



class State():
    def __init__(self, position_world, velocity_body, orientation_world, angular_velocity_body):
        self.position_world = position_world
        self.velocity_body = velocity_body
        self.orientation_world = orientation_world
        self.angular_velocity_body = angular_velocity_body


    @staticmethod
    def from_odometry_msg(msg: Odometry):
        """
        Utility method to convert an Odometry message to a State object.
        """
        # Extract the position from the pose
        position_world = np.array([msg.pose.pose.position.x,
                                   msg.pose.pose.position.y,
                                   msg.pose.pose.position.z])
        
        # Extract the orientation from the pose (quaternion)
        orientation_world = np.quaternion([
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z
        ])
        
        # Extract the linear velocity from the twist
        velocity_body = np.array([msg.twist.twist.linear.x,
                                  msg.twist.twist.linear.y,
                                  msg.twist.twist.linear.z])
        
        # Extract the angular velocity from the twist
        angular_velocity_body = np.array([msg.twist.twist.angular.x,
                                          msg.twist.twist.angular.y,
                                          msg.twist.twist.angular.z])
        
        return State(position_world, velocity_body, orientation_world, angular_velocity_body)

   

def update(PIDController):
    if self.cur_state is None or self.ref_state is None:
        return

    self.publish_control()
def main(args=None):
    rclpy.init(args=args)

    node = Controller(
        kP={'vel': 1.0, 'pos': 1.0, 'wrench': 1.0}, 
        kD={'vel': 0.1, 'pos': 0.1, 'wrench': 0.1}, 
        kI={'vel': 0.01, 'pos': 0.01, 'wrench': 0.01}, 
        iSum={'vel': 0.0, 'pos': 0.0, 'wrench': 0.0},
        max_signal=np.array([[100, 100, 100], [10, 10, 10]]), 
        max_i=np.array([[10, 10, 10], [1, 1, 1]])
    )

    timer_period = 0.1  # seconds
    # call update every timer
    node.create_timer(timer_period, node.update)
    # Spin the node to keep it active and process messages

    rclpy.spin(node)


    rclpy.shutdown()


if __name__ == "__main__":
    main()
