import rclpy
from rclpy.node import Node


class ObjectLocalizer(Node):
    def __init__(self, model):
        super().__init__(f"{model} detection localizer")
