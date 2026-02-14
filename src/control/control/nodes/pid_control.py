from rclpy.node import Node

class PIDControl(Node):
    def __init__(self):
        ## should be subbed to topics /world/pose, /robot/pose, /desired/pose
        ## outputs thrusts to hardware
        pass