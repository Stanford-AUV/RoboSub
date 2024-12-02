"""
Logger Node.

Dependencies:
    matplotlib.pyplot
    rclpy.node.Node
    signal

Author:
    Ali Ahmad
"""

import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from control.utils.state import State
from nav_msgs.msg import Odometry
import signal
import numpy as np

class Logger(Node):

    def __init__(self):
        super().__init__('logger')
        self.create_subscription(Odometry, 'odometry', self.log_position, 10)
        self.positions = []

        signal.signal(signal.SIGINT, self.handle_shutdown)

    def log_position(self, msg):
        timestamp = msg.header.stamp.sec
        state = State.from_msg(msg)
        self.positions.append((timestamp, state.position))
    
    def handle_shutdown(self, signum, frame):
        self.get_logger().info("Node shutting down. Creating plot...")
        self.plot()
        rclpy.shutdown()

    def plot(self):
        if not self.positions:
            self.get_logger().info("No data to plot.")
            return

        timestamps, positions = zip(*self.positions)
        displacements = 
        plt.figure()
        plt.plot(timestamps, positions)