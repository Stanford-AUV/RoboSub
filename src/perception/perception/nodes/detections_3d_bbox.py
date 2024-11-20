import rclpy
from rclpy.node import Node

from perception.utils.bounding_box import oriented_bounding_box


class Detections3DBBoxNode(Node):
    def __init__(self):
        super().__init__("detections_3d_bbox")


def main(args=None):
    rclpy.init(args=args)
    node = Detections3DBBoxNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
