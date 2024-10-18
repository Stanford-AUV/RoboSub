import cv2
import random
import numpy as np
import os
from typing import Tuple
from datetime import datetime

import rclpy
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.lifecycle import LifecycleState

import message_filters
from cv_bridge import CvBridge
from ultralytics.utils.plotting import Annotator, colors

from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from yolov8_msgs.msg import BoundingBox2D
from yolov8_msgs.msg import Detection
from yolov8_msgs.msg import DetectionArray
import os
import glob
from pathlib import Path


class DataNode(LifecycleNode):

    def __init__(self) -> None:
        super().__init__("data_node")

        self._class_to_color = {}
        self.cv_bridge = CvBridge()

        # params
        self.declare_parameter("image_reliability",
                               QoSReliabilityPolicy.BEST_EFFORT)
        self.declare_parameter("max_files", 2000)

        self.get_logger().info("Data node created")
        self.output_dir = "./raw_images"
        self.last_save_time = 0
        os.makedirs(self.output_dir, exist_ok=True)


    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Configuring {self.get_name()}")

        self.image_qos_profile = QoSProfile(
            reliability=self.get_parameter(
                "image_reliability").get_parameter_value().integer_value,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Activating {self.get_name()}")

        # subs
        self.image_sub = self.create_subscription(
            Image, "/oak/rgb/image_raw", self.image_cb, qos_profile=self.image_qos_profile)

        print("###############")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Deactivating {self.get_name()}")

        self.destroy_subscription(self.image_sub.sub)

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Cleaning up {self.get_name()}")

        return TransitionCallbackReturn.SUCCESS


    def maintain_image_limit(self, folder_path, max_files=100):
        # Convert folder_path to Path object
        folder_path = Path(folder_path)

        # List all image files in the directory
        files = list(folder_path.glob('*'))  # Adjust pattern if needed, e.g., '*.jpg'
        
        # Check if there are more files than the allowed limit

        if len(files) > max_files:
            # Sort files by modification time (oldest first)
            files.sort(key=lambda x: x.stat().st_mtime)
            
            # Calculate the number of files to delete
            files_to_delete = files[:len(files) - max_files]
            
            # Delete the oldest files
            for file in files_to_delete:
                try:
                    file.unlink()
                    print(f"Deleted {file}")
                except Exception as e:
                    print(f"Error deleting {file}: {e}")
                    
    def image_cb(self, msg: Image) -> None:
        current_time = self.get_clock().now().to_msg().sec
        if current_time - self.last_save_time >= 1.0:
            self.last_save_time = current_time
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg)
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = os.path.join(self.output_dir, f'{timestamp}.jpg')
            cv2.imwrite(filename, cv_image)
            print(f"Wrote to {filename}")
            self.maintain_image_limit(self.output_dir, self.get_parameter("max_files").get_parameter_value().integer_value,)
            

def main():
    rclpy.init()
    node = DataNode()
    node.trigger_configure()
    node.trigger_activate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
