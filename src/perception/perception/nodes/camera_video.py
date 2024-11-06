#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import depthai as dai
import time
import numpy as np
import cv2  # Import OpenCV


class CameraPublisher(Node):
    def __init__(self):
        super().__init__("camera_publisher")
        self.get_logger().info("DepthAI camera publisher node has been created!")

        self.bridge = CvBridge()

        # Create DepthAI pipeline
        self.pipeline = dai.Pipeline()

        # Define sources and outputs for RGB and depth cameras
        cam_rgb = self.pipeline.createColorCamera()
        cam_left = self.pipeline.createMonoCamera()
        cam_right = self.pipeline.createMonoCamera()
        cam_depth = self.pipeline.createStereoDepth()

        xout_rgb = self.pipeline.createXLinkOut()
        xout_depth = self.pipeline.createXLinkOut()

        # Properties for the RGB camera
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        xout_rgb.setStreamName("rgb")

        # Properties for the mono cameras for depth computation
        cam_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        cam_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        cam_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        cam_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        # Depth configuration
        cam_depth.setConfidenceThreshold(200)
        xout_depth.setStreamName("depth")

        # Linking
        cam_rgb.video.link(xout_rgb.input)
        cam_left.out.link(cam_depth.left)
        cam_right.out.link(cam_depth.right)
        cam_depth.depth.link(xout_depth.input)

        # Start the device and create the video stream
        self.device = dai.Device(self.pipeline)

        # Output queues for the RGB and depth streams
        self.q_rgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        self.q_depth = self.device.getOutputQueue(
            name="depth", maxSize=4, blocking=False
        )

        # ROS2 publishers
        self.pub_rgb = self.create_publisher(Image, "oak/rgb/image_raw", 10)
        self.pub_depth = self.create_publisher(Image, "oak/depth/image_raw", 10)

    def run(self):
        count = 0
        while True:
            # Get RGB frame
            in_rgb = self.q_rgb.get()  # Blocking call, waits for new data
            frame_rgb = in_rgb.getCvFrame()

            # Get depth frame
            in_depth = self.q_depth.get()
            frame_depth = in_depth.getFrame()  # Depth data is in millimeters

            # Convert depth to 16-bit grayscale (as Image expects mono16 encoding)
            depth_image = cv2.normalize(
                frame_depth, None, 0, 65535, cv2.NORM_MINMAX
            ).astype(np.uint16)

            # Publish RGB and depth frames
            self.pub_rgb.publish(self.bridge.cv2_to_imgmsg(frame_rgb, "bgr8"))
            self.pub_depth.publish(self.bridge.cv2_to_imgmsg(depth_image, "mono16"))

            # Log every 100 frames
            if count % 100 == 0:
                self.get_logger().info(f"Published frame {count}")

            count += 1

            # Sleep to limit publishing rate
            time.sleep(1 / 30)  # Assuming 30 FPS, adjust as necessary


def main(args=None):
    rclpy.init(args=args)

    cp = CameraPublisher()

    print("Publishing DepthAI camera frames...")
    cp.run()

    cp.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
