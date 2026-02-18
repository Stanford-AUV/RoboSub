from rclpy.node import Node
from sensor_msgs.msg import Imu
from msgs.msg import DVLData, DVLBeam, DVLTarget, DVLVelocity


class EKF(Node):
    def __init__(self):
        super.__init__('ekf')
        
        self.imu_sub = self.create_subscription(Imu, "/imu/data", self.imu_cb, 50)
        self.odom_sub = self.create_subscription(DVLData, "/wheel/odom", self.odom_cb, 20)
        self.odom_sub = self.create_subscription(Odometry, "/wheel/odom", self.odom_cb, 20)

        # gets information from IMU, pulls sensor covariance, packtrack 
        # publish /robot/pose and
        
        
        ### gets subs from IMU, pulls covariance from sensors, backtrack the file pkg
        ## pubs to /robot/pose and /world/pose
        pass
