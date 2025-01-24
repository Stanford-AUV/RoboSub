import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import depthai as dai
import numpy as np


class Camera(Node):
    def __init__(self, device):
        super().__init__("camera")
        self.get_logger().info("DepthAI camera publisher node has been created!")

        self.device = device
        self.bridge = CvBridge()

        # ROS2 publishers
        self.rgb_pub = self.create_publisher(Image, "oak/rgb/image_raw", 10)
        self.depth_pub = self.create_publisher(Image, "oak/depth/image_raw", 10)

        # Store the latest packets
        self.latest_packet = {"rgb": None, "disp": None}

        # Timer for publishing frames
        self.timer = self.create_timer(0.033, self.process_and_publish_frames)  # 30 FPS

    def process_and_publish_frames(self):
        # Get queue events
        queue_events = self.device.getQueueEvents(("rgb", "disp"))
        for queue_name in queue_events:
            packets = self.device.getOutputQueue(queue_name).tryGetAll()
            if len(packets) > 0:
                self.latest_packet[queue_name] = packets[-1]

        # Check if both RGB and depth frames are available
        if (
            self.latest_packet["rgb"] is not None
            and self.latest_packet["disp"] is not None
        ):
            frame_rgb = self.latest_packet["rgb"].getCvFrame()
            frame_disp = self.latest_packet["disp"].getFrame()

            # Publish frames and log their shapes
            self.get_logger().info(f"Publishing RGB frame of shape: {frame_rgb.shape}")
            self.get_logger().info(
                f"Publishing Depth frame of shape: {frame_disp.shape}"
            )

            self.rgb_pub.publish(self.bridge.cv2_to_imgmsg(frame_rgb, "bgr8"))
            self.depth_pub.publish(
                self.bridge.cv2_to_imgmsg(frame_disp, encoding="passthrough")
            )


def create_pipeline(fps):
    # The disparity is computed at this resolution, then upscaled to RGB resolution
    # monoResolution = dai.MonoCameraProperties.SensorResolution.THE_720_P

    # Create pipeline
    pipeline = dai.Pipeline()

    # Define sources and outputs
    # camRgb = pipeline.create(dai.node.ColorCamera)
    left = pipeline.create(dai.node.MonoCamera)
    right = pipeline.create(dai.node.MonoCamera)
    stereo = pipeline.create(dai.node.StereoDepth)

    # rgbOut = pipeline.create(dai.node.XLinkOut)
    # disparityOut = pipeline.create(dai.node.XLinkOut)

    # rgbOut.setStreamName("rgb")
    # disparityOut.setStreamName("disp")

    # RGB Camera Properties
    # camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
    # camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    # camRgb.setFps(fps)

    # Mono Cameras Properties
    # left.setResolution(monoResolution)
    # left.setBoardSocket(dai.CameraBoardSocket.LEFT)
    # left.setFps(fps)

    # right.setResolution(monoResolution)
    # right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
    # right.setFps(fps)

    # StereoDepth Properties
    # stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    # stereo.setLeftRightCheck(True)  # Enable Left-Right Check to reduce artifacts
    # stereo.setSubpixel(True)  # Improves depth precision
    # stereo.setDepthAlign(
    #     dai.CameraBoardSocket.RGB
    # )  # Align depth map with RGB perspective

    # Filters
    # stereo.initialConfig.setSpeckleFilter(enable=True, speckleRange=50)
    # stereo.initialConfig.setTemporalFilter(enable=True, alpha=0.4, delta=50)
    # stereo.initialConfig.setSpatialFilter(
    #     enable=True, holeFillingRadius=2, numIterations=1
    # )
    # stereo.initialConfig.setDepthThreshold(
    #     minRange=300, maxRange=12000
    # )  # 30 cm to 12 m
    # stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)

    # Linking
    # camRgb.video.link(rgbOut.input)  # Link RGB camera output to RGB XLinkOut
    left.out.link(stereo.left)  # Link left mono camera to StereoDepth
    right.out.link(stereo.right)  # Link right mono camera to StereoDepth
    # stereo.disparity.link(
    #     disparityOut.input
    # )  # Link stereo depth output to disparity XLinkOut

    return pipeline


def main():
    rclpy.init()

    # Set FPS for the cameras
    fps = 30

    # Create DepthAI pipeline and device
    pipeline = create_pipeline(fps)
    device = dai.Device(pipeline)

    # Set fixed focus for RGB camera based on calibration data
    try:
        calibData = device.readCalibration2()
        lensPosition = calibData.getLensPosition(dai.CameraBoardSocket.RGB)
        if lensPosition:
            device.getInputQueue("rgb").send(
                dai.CameraControl().setManualFocus(lensPosition)
            )
    except Exception as e:
        raise RuntimeError("Failed to read calibration data or set focus.") from e

    # Create and spin the Camera node
    camera_node = Camera(device)
    rclpy.spin(camera_node)

    # Clean up
    camera_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
