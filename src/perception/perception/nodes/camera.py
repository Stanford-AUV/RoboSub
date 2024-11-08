import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import depthai as dai
import time


class Camera(Node):
    def __init__(self):
        super().__init__("camera")
        self.get_logger().info("DepthAI camera publisher node has been created!")

        self.pipeline = dai.Pipeline()
        cam_rgb = self.pipeline.createColorCamera()
        monoLeft = self.pipeline.create(dai.node.MonoCamera)
        monoRight = self.pipeline.create(dai.node.MonoCamera)
        cam_depth = self.pipeline.createStereoDepth()
        xout_rgb = self.pipeline.createXLinkOut()
        xout_depth = self.pipeline.createXLinkOut()

        # Properties for the RGB camera
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        xout_rgb.setStreamName("rgb")

        # Mono camera properties
        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setCamera("left")
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setCamera("right")

        # Properties for the depth camera
        cam_depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        cam_depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
        cam_depth.setLeftRightCheck(True)
        cam_depth.setExtendedDisparity(False)
        cam_depth.setSubpixel(False)
        xout_depth.setStreamName("depth")

        # Linking
        monoLeft.out.link(cam_depth.left)
        monoRight.out.link(cam_depth.right)
        cam_rgb.video.link(xout_rgb.input)
        cam_depth.disparity.link(xout_depth.input)

        # Start the device and create the video stream
        self.device = dai.Device(self.pipeline)

        self.q_rgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        self.q_depth = self.device.getOutputQueue(
            name="depth", maxSize=4, blocking=False
        )

        # ROS2 publisher
        self.bridge = CvBridge()
        self.rgb_pub = self.create_publisher(Image, "oak/rgb/image_raw", 10)
        self.depth_pub = self.create_publisher(Image, "oak/depth/image_raw", 10)

        count = 0
        while True:
            in_rgb = self.q_rgb.get()
            rgb_frame = in_rgb.getCvFrame()
            self.rgb_pub.publish(self.bridge.cv2_to_imgmsg(rgb_frame, "bgr8"))

            in_depth = self.q_depth.get()
            depth_frame = in_depth.getFrame()

            self.depth_pub.publish(
                self.bridge.cv2_to_imgmsg(depth_frame, encoding="passthrough")
            )

            # Log every 100 frames
            if count % 100 == 0:
                self.get_logger().info(f"Published frame {count}")

            count += 1

            # Sleep to limit publishing rate
            time.sleep(1 / 30)  # Assuming 30 FPS, adjust as necessary


def main(args=None):
    rclpy.init(args=args)
    node = Camera()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
