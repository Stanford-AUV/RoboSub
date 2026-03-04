#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import yaml
import os
from cv_bridge import CvBridge


class GenericCameraNode(Node):
    def __init__(self, node_name, path):
        super().__init__(node_name)

        self.bridge = CvBridge()
        self.rgb_pubs = {}
        self.depth_pubs = {}
        self.yolo_rgb = None  # Annotated RGB image from YOLO detections

        self.devices = {}  # holds device objects
        self.queues = {}  # holds output queues

        self._camera_info = {}
        self.path = path

        
    def publish_frames(self):
        raise NotImplementedError("Implement this (publish_frames)!")
    
    def load_cameras_yaml(self):
        if not os.path.exists(self.path):
            raise FileNotFoundError("Noooooo! No yaml path exists :(")

        with open(self.path, "r") as f:
            data = yaml.safe_load(f)

        return data

    def get_cam_data(self, data, name):
        cam_data = {}
        for key, value in data.items():
            if name in key:
                cam_data[key] = value
        return cam_data
