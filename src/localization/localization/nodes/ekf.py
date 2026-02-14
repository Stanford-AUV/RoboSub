from rclpy.node import Node

class EKF(Node):
    def __init__(self):
        ### gets subs from IMU, pulls covariance from sensors, backtrack the file pkg
        ## pubs to /robot/pose and /world/pose
        pass
