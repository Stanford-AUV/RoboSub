#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import cv2
import os
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time


class ImagePublisher(Node):
    def __init__(self, file_path):
        super().__init__("image_publisher")
        self.get_logger().info("Video publisher node has been created!")

        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(file_path)
        self.pub = self.create_publisher(Image, "oak/rgb/image_raw", 10)

        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open video file: {file_path}")
            sys.exit(1)

        # Get the frame rate of the video
        self.fps = self.cap.get(cv2.CAP_PROP_FPS)
        if self.fps == 0:
            self.get_logger().error("Failed to retrieve FPS. Defaulting to 30 FPS.")
            self.fps = 30  # Default frame rate if not detected

        self.frame_duration = 1.0 / self.fps  # Time between frames

    def run(self):
        count = 0
        while self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                # Publish frame
                self.pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

                # Log every 100 frames
                if count % 100 == 0:
                    self.get_logger().info(f"Published frame {count}")

                count += 1

                # Sleep to match the frame rate of the video
                time.sleep(self.frame_duration)
            else:
                # Loop the video if it reaches the end
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                count = 0

        self.cap.release()


def main(args=None):
    rclpy.init(args=args)

    file_path = "videoplayback.mp4"
    ip = ImagePublisher("/workspaces/RoboSub/src/perception/videoplayback.mp4")

    print("Publishing video frames...")
    ip.run()

    ip.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
