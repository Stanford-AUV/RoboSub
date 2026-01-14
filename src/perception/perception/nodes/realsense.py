#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.time import Time
import numpy as np
import os
import pyrealsense2 as rs
from generic_camera import GenericCameraNode


class Device:
    def __init__(self, pipeline, pipeline_profile, product_line):
        self.pipeline = pipeline
        self.pipeline_profile = pipeline_profile
        self.product_line = product_line


class RealsenseNode(GenericCameraNode):
    def __init__(self, path):
        super().__init__("realsense_node", path)

        self.path = path
        self._camera_info = self.get_cam_data(self.load_cameras_yaml(), "realsense")

        for key, cam_cfg in self._camera_info.items():
            cam_cfg = cam_cfg.get("ros__parameters")

            self.get_logger().info(
                f"Starting Realsense camera: {key} "
                f"(Serial number: {cam_cfg.get('camera').get('i_serial_number')})"
            )

            rgb_pub = self.create_publisher(Image, f"/camera/{key}/rgb", 10)
            self.rgb_pubs[key] = rgb_pub

            depth_pub = self.create_publisher(Image, f"/camera/{key}/depth", 10)
            self.depth_pubs[key] = depth_pub

            pipeline = rs.pipeline()
            device_serial = cam_cfg.get("camera").get("i_serial_number")
            product_line = cam_cfg.get("camera").get("i_product_line")

            c = rs.config()
            c.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 6)
            c.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 6)

            c.enable_device(device_serial)
            pipeline_profile = pipeline.start(c)
            self.devices[key] = Device(pipeline, pipeline_profile, product_line)

        self.create_timer(1.0 / 30.0, self.publish_frames)  # 30 FPS

    def publish_frames(self):
        for key, device in self.devices.items():

            frameset = device.pipeline.poll_for_frames()

            if not frameset:
                continue

            depth_frame = frameset.get_depth_frame()
            rgb_frame = frameset.get_color_frame()
            timestamp = self.get_clock().now().to_msg()

            if rgb_frame is not None:
                ros_rgb = self.rs_color_to_ros_image(key, timestamp, rgb_frame)
                self.rgb_pubs[key].publish(ros_rgb)

            if depth_frame is not None:
                ros_depth = self.rs_depth_to_ros_image(key, timestamp, depth_frame)
                self.depth_pubs[key].publish(ros_depth)

    def rs_color_to_ros_image(self, key, timestamp, color_frame):
        img = np.asanyarray(color_frame.get_data())

        ros_img = Image()
        ros_img.height = img.shape[0]
        ros_img.width = img.shape[1]
        ros_img.encoding = "rgb8"
        ros_img.is_bigendian = False
        ros_img.step = img.shape[1] * 3
        ros_img.data = img.tobytes()
        ros_img.header.stamp = timestamp
        ros_img.header.frame_id = f"{key}_color_optical_frame"

        return ros_img

    def rs_depth_to_ros_image(self, key, timestamp, depth_frame):
        depth = np.asanyarray(depth_frame.get_data())

        ros_img = Image()
        ros_img.height = depth.shape[0]
        ros_img.width = depth.shape[1]
        ros_img.encoding = "16UC1"  # VERY IMPORTANT
        ros_img.is_bigendian = False
        ros_img.step = depth.shape[1] * 2
        ros_img.data = depth.tobytes()
        ros_img.header.stamp = timestamp
        ros_img.header.frame_id = f"{key}_depth_optical_frame"

        return ros_img


def main(args=None):
    rclpy.init(args=args)

    SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
    yaml_path = os.path.join(SCRIPT_DIR, "..", "cameras.yaml")
    yaml_path = os.path.abspath(yaml_path)

    node = RealsenseNode(yaml_path)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
