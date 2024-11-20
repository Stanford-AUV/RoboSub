"""
Logger Node.

Dependencies:
    matplotlib.pyplot
    rclpy.node.Node
    signal

Author:
    Ali Ahmad
"""

import matplotlib.pyplot as plt

from control.utils.state import State

from nav_msgs.msg import Odometry

import rclpy
from rclpy.node import Node

import signal

class Logger(Node):

    def __init__(self):
        super().__init__('logger')
        self.create_subscription(Odometry, 'odometry', self.log_path, 10)
        self.positions = []
        self.orientations = []
    