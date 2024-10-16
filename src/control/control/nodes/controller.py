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

Version:
    1.0.0
"""

import threading

from control.utils.utils import quat_to_axis_angle, qvmul, to_np

from geometry_msgs.msg import WrenchStamped

from nav_msgs.msg import Odometry

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
                 kp: np.array,
                 kd: np.array,
                 ki: np.array,
                 max_signal: np.ndarray,
                 max_i: np.ndarray):
        """
        Initialize the controller node.

        We initialize the controller node with subscribers to the state and
        reference topics, and a publisher to the desired_wrench topic. We
        initialize the PID controller with given parameters and a lock for
        thread safety.

        Parameters
        ----------
        kp: np.ndarray (2x3)
            The proportional gains for the controller (pos, ang).
        kd: np.ndarray (2x3)
            The derivative gains for the controller (pos, ang).
        ki: np.ndarray (2x3)
            The integral gains for the controller (pos, ang).
        ceil: np.ndarray (2x3)
            The saturation limits for the controller.
        max_i: np.ndarray (2x3)
            The integral saturation limits for the controller.
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

        self.time = self.get_clock().now()

        self.kP_pos = kp[0]
        self.kP_ang = kp[1]
        self.kD_pos = kd[0]
        self.kD_ang = kd[1]
        self.kI_pos = ki[0]
        self.kI_ang = ki[1]
        self.max_signal = max_signal
        self.max_i = max_i

        self.state = None
        self.reference = None
        self.integral_pos = 0
        self.integral_ang = 0

    def state_callback(self, msg: Odometry):
        with self.lock:
            self.state = to_np(msg)
            timestamp = Time(
                seconds=msg.header.stamp.sec,
                nanoseconds=msg.header.stamp.nanosec,
                clock_type=self.get_clock().clock_type
            )
            self.dt = (timestamp - self.time).nanoseconds / 1e6
            self.time = timestamp
            self.update()

    def reference_callback(self, msg: Odometry):
        with self.lock:
            self.reference = to_np(msg)

    def update(self):
        r_W = self.state[0:3]  # position, world frame
        v_B = self.state[3:6]  # velocity, body frame
        q_W = quaternion(self.state[6:10])  # orientation, world frame
        w_B = self.state[10:13]  # angular velocity, body frame

        r_W_ref = self.reference[0:3]  # position, world frame
        v_B_ref = self.reference[3:6]  # velocity, body frame
        q_W_ref = quaternion(self.reference[6:10])  # orientation, world frame
        w_B_ref = self.reference[10:13]  # angular velocity, body frame

        # pose error
        error_r_W = r_W_ref - r_W

        q_W_inv = np.conjugate(q_W)
        q_W_ref_inv = np.conjugate(q_W_ref)
        error_q_W = q_W_inv * q_W_ref_inv
        error_q_W = quat_to_axis_angle(error_q_W)

        # velocity error
        error_v_B = v_B_ref - v_B
        error_w_B = w_B_ref - w_B

        # integral term
        self.integral_pos = np.clip(
            (self.integral_pos + error_r_W + error_v_B) * self.dt,
            -self.max_i[0],
            self.max_i[0]
        )
        self.integral_ang = np.clip(
            (self.integral_ang + error_q_W + error_w_B) * self.dt,
            -self.max_i[1],
            self.max_i[1]
        )

        # force on the body (in the body frame)
        F_W = self.kP_pos * error_r_W + self.kD_pos * error_v_B + self.kI_pos * self.integral_pos
        F_B = qvmul(q_W_inv, F_W)

    def reset(self):
        self.get_logger.info('Resetting controller...')
        with self.lock:
            self.integral = np.zeros(6)


def main(args=None):
    rclpy.init(args=args)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
