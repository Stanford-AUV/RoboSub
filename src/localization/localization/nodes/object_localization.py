from rclpy.node import Node

class ObjectLocalization(Node):
    def __init__(self):
        ### ideally this pulls camera bounding boxes, opens cam yaml to get intrinsic / extrinsic matrix and converts to world coords
        ### pubs to /<object_name>/robot/pose, can get /<object_name>/world/pose if sub to /world/pose
        pass
