#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import depthai as dai
import time


class CameraPublisher(Node):
    def __init__(self):
        super().__init__("camera_publisher")
        self.get_logger().info("DepthAI camera publisher node has been created!")

        # Create DepthAI pipeline
        self.pipeline = dai.Pipeline()

        # Define sources and outputs
        cam_rgb = self.pipeline.createColorCamera()
        xout_rgb = self.pipeline.createXLinkOut()

        # Properties for the RGB camera
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        xout_rgb.setStreamName("rgb")

        # Linking
        cam_rgb.video.link(xout_rgb.input)

        # Start the device and create the video stream
        self.device = dai.Device(self.pipeline)

        # Output queue for the RGB stream
        self.q_rgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

        # ROS2 publisher
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, "oak/rgb/image_raw", 10)

    def run(self):
        count = 0
        while True:
            # Get the frame from the camera
            in_rgb = (
                self.q_rgb.get()
            )  # Blocking call, will wait until a new data is available
            frame = in_rgb.getCvFrame()

            # Publish the frame
            self.pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

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
