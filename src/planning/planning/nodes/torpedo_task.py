#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int16
from msgs.msg import Waypoint
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy


class TorpedoTask(Node):
    """
    Listens for a follow waypoint (e.g., subject="torpedo_hole"),
    waits for `waypoint/object_reached` from PathTracker, and then fires the torpedo.

    Publishes to the "torpedo" topic as Int16:
      - Positive value => left torpedo ("nagasaki" in Arduino code)
      - Negative value => right torpedo ("hiroshima" in Arduino code)
    """

    def __init__(self):
        super().__init__('torpedo_task')

        # Parameters
        self.declare_parameter('target_subject', 'torpedo_hole')
        self.declare_parameter('fire_side', 'left')  # "left" or "right"

        self.target_subject = self.get_parameter('target_subject').get_parameter_value().string_value
        self.fire_side = self.get_parameter('fire_side').get_parameter_value().string_value.lower()

        # State
        self.current_subject = None
        self.already_fired = False

        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL   # latch last msg
        qos.reliability = QoSReliabilityPolicy.RELIABLE

        # Subscribers
        self.create_subscription(Waypoint, 'waypoint', self._waypoint_cb, qos)
        self.create_subscription(Bool, 'waypoint/object_reached', self._object_reached_cb, qos)

        # Publisher to Arduino
        self.torpedo_pub = self.create_publisher(Int16, 'torpedo', 10)

        self.get_logger().info(
            f"TorpedoTask armed: target_subject='{self.target_subject}', fire_side='{self.fire_side}'"
        )

    def _waypoint_cb(self, msg: Waypoint):
        # Remember what we're currently trying to reach
        self.current_subject = msg.subject
        self.get_logger().debug(f"New waypoint: purpose={msg.purpose}, subject={msg.subject}")
        # Reset fire state if new matching target comes in
        if self.current_subject == self.target_subject:
            self.already_fired = False

    def _object_reached_cb(self, msg: Bool):
        if not msg.data:
            return  # Only care about True events

        if self.current_subject != self.target_subject:
            return  # Not our target object

        if self.already_fired:
            return  # Prevent double-fire

        # Decide which side to fire
        if self.fire_side == 'left':
            value = 1   # triggers "nagasaki" in Arduino
        elif self.fire_side == 'right':
            value = -1  # triggers "hiroshima" in Arduino
        else:
            self.get_logger().error(f"Invalid fire_side: {self.fire_side}")
            return

        self.get_logger().info(
            f"Target '{self.target_subject}' reached and stable. Firing {self.fire_side} torpedo!"
        )
        self.torpedo_pub.publish(Int16(data=value))
        self.already_fired = True


def main(args=None):
    rclpy.init(args=args)
    node = TorpedoTask()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
