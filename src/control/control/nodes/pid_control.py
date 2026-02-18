import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped
from control.utils.pid import PID
from control.utils.state import State

class PIDControl(Node):
    def __init__(self, kP_pos, kI_pos, kD_pos, kP_orie, kI_orie, 
                 kD_orie, max_integral_pos, max_integral_orie):
        
        self.PID = PID(kP_position=kP_pos, kI_position=kI_pos, kD_position=kD_pos,
                       kP_orientation=kP_orie, kI_orientation=kI_orie, kD_orientation=kD_orie,
                       max_integral_position=max_integral_pos, max_integral_orientation=max_integral_orie)
        
        self.curr_state = None
        self.desired_state = None

        self.current_state_subscriber = self.create_subscription(
            Odometry, "/world/pose", self.update_curr_state, 10
        )

        self.desired_state_subscriber = self.create_subscription(
            Odometry, "/desired/pose", self.update_des_state, 10
        )

        self.wrench_publisher = self.create_publisher(
            WrenchStamped, "/wrench", 10
        )

        self.dt = 1.0 / 60.0 # 60 FPS
        self.create_timer(self.dt, self.publish_pid)  
    
    def reset(self):
        self.PID.reset()

    def update_curr_state(self, msg: Odometry):
        self.curr_state = State.from_odometry_msg(msg)

    def update_des_state(self, msg: Odometry):
        self.desired_state = State.from_odometry_msg(msg)
    
    def pid_loop(self):
        if self.curr_state is None or self.desired_state is None:
            return
        wrench_msg = self.PID.update(self.curr_state, self.desired_state, self.dt)
        self.wrench_publisher.publish(wrench_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PIDControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
