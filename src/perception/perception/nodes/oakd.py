import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class OakDSubscriber(Node):

    def __init__(self):
        super().__init__("oakd_subscriber")
        self.rgb_subscription_ = self.create_subscription(
            Image, "oak/rgb/image_raw", self.rgb_callback, 10
        )
        self.stereo_subscription_ = self.create_subscription(
            Image, "oak/stereo/image_raw", self.stereo_callback, 10
        )

    def rgb_callback(self, msg):
        self.get_logger().info("Subscribed to RGB")

    def stereo_callback(self, msg):
        self.get_logger().info("Subscribed to stereo")


def main(args=None):
    rclpy.init(args=args)

    oakd_subscriber = OakDSubscriber()

    rclpy.spin(oakd_subscriber)

    # Destroy the node explicitly
    oakd_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
