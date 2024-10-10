"""
Controller node for the control system.

Author:
    Ali Ahmad

Version:
    1.0.0
"""
from control.utils.pid import PID
import numpy as np
import rclpy
from geometry_msgs.msg import Wrench, WrenchStamped
from rclpy import Parameter
from rclpy.node import Node
from msgs.msg import ThrustsStamped

class Controller(Node):
    """The Controller Node."""

    def __init__(self):
        """Initialize the controller node."""
        pass

