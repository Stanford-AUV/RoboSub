# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import depthai as dai


# class Camera(Node):
#     def __init__(self, device):
#         super().__init__("camera")
#         self.get_logger().info("DepthAI camera publisher node has been created!")

#         self.device = device

#         # ROS2 publisher
#         self.bridge = CvBridge()
#         self.rgb_pub = self.create_publisher(Image, "oak/rgb/image_raw", 10)
#         self.depth_pub = self.create_publisher(Image, "oak/depth/image_raw", 10)

#         device = self.device
#         latestPacket = {}
#         latestPacket["rgb"] = None
#         latestPacket["disp"] = None

#         while True:
#             queueEvents = device.getQueueEvents(("rgb", "disp"))
#             for queueName in queueEvents:
#                 packets = device.getOutputQueue(queueName).tryGetAll()
#                 if len(packets) > 0:
#                     latestPacket[queueName] = packets[-1]

#             if latestPacket["rgb"] is not None and latestPacket["disp"] is not None:
#                 frameRgb = latestPacket["rgb"].getCvFrame()
#                 frameDisp = latestPacket["disp"].getFrame()
#                 # maxDisparity = stereo.initialConfig.getMaxDisparity()
#                 # TODO: Move to view cam
#                 # # Optional, extend range 0..95 -> 0..255, for a better visualisation
#                 # if 1:
#                 #     frameDisp = (frameDisp * 255.0 / maxDisparity).astype(np.uint8)
#                 # # Optional, apply false colorization
#                 # if 1:
#                 #     frameDisp = cv2.applyColorMap(frameDisp, cv2.COLORMAP_HOT)
#                 # frameDisp = np.ascontiguousarray(frameDisp)
#                 # cv2.imshow(depthWindowName, frameDisp)
#                 self.rgb_pub.publish(self.bridge.cv2_to_imgmsg(frameRgb, "bgr8"))
#                 self.depth_pub.publish(
#                     self.bridge.cv2_to_imgmsg(frameDisp, encoding="passthrough")
#                 )


# def setup_device():
#     fps = 30

#     # The disparity is computed at this resolution, then upscaled to RGB resolution
#     monoResolution = dai.MonoCameraProperties.SensorResolution.THE_720_P

#     # Create pipeline
#     pipeline = dai.Pipeline()
#     device = dai.Device()
#     queueNames = []

#     # Define sources and outputs
#     camRgb = pipeline.create(dai.node.Camera)
#     left = pipeline.create(dai.node.MonoCamera)
#     right = pipeline.create(dai.node.MonoCamera)
#     stereo = pipeline.create(dai.node.StereoDepth)

#     rgbOut = pipeline.create(dai.node.XLinkOut)
#     disparityOut = pipeline.create(dai.node.XLinkOut)

#     rgbOut.setStreamName("rgb")
#     queueNames.append("rgb")
#     disparityOut.setStreamName("disp")
#     queueNames.append("disp")

#     # Properties
#     rgbCamSocket = dai.CameraBoardSocket.CAM_A

#     camRgb.setBoardSocket(rgbCamSocket)
#     camRgb.setSize(1280, 720)
#     camRgb.setFps(fps)

#     # For now, RGB needs fixed focus to properly align with depth.
#     # This value was used during calibration
#     try:
#         calibData = device.readCalibration2()
#         lensPosition = calibData.getLensPosition(rgbCamSocket)
#         if lensPosition:
#             camRgb.initialControl.setManualFocus(lensPosition)
#     except:
#         raise
#     left.setResolution(monoResolution)
#     left.setCamera("left")
#     left.setFps(fps)
#     right.setResolution(monoResolution)
#     right.setCamera("right")
#     right.setFps(fps)

#     stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
#     # LR-check is required for depth alignment
#     stereo.setLeftRightCheck(True)
#     stereo.setDepthAlign(rgbCamSocket)

#     # Linking
#     camRgb.video.link(rgbOut.input)
#     left.out.link(stereo.left)
#     right.out.link(stereo.right)
#     stereo.disparity.link(disparityOut.input)

#     camRgb.setMeshSource(dai.CameraProperties.WarpMeshSource.CALIBRATION)

#     # Connect to device and start pipeline
#     return device, pipeline


# def main(args=None):
#     rclpy.init(args=args)
#     device, pipeline = setup_device()
#     with device:
#         device.startPipeline(pipeline)
#         node = Camera(device)
#         rclpy.spin(node)
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import depthai as dai
import cv2


class OakCameraPublisher(Node):
    def __init__(self):
        super().__init__("oak_camera_publisher")
        self.bridge = CvBridge()

        # Publishers
        self.rgb_pub = self.create_publisher(Image, "oak/rgb/image_raw", 10)
        self.disp_pub = self.create_publisher(Image, "oak/depth/image_raw", 10)

        # Initialize DepthAI pipeline/device
        self.device, self.queues = self.setup_device()

        # Timer for frame polling
        self.create_timer(1.0 / 30.0, self.publish_frames)

    def setup_device(self):
        # Create pipeline
        pipeline = dai.Pipeline()
        fps = 30

        # Define sources and outputs
        camRgb = pipeline.create(dai.node.ColorCamera)
        left = pipeline.create(dai.node.MonoCamera)
        right = pipeline.create(dai.node.MonoCamera)
        stereo = pipeline.create(dai.node.StereoDepth)

        rgbOut = pipeline.create(dai.node.XLinkOut)
        dispOut = pipeline.create(dai.node.XLinkOut)

        rgbOut.setStreamName("rgb")
        dispOut.setStreamName("disp")

        camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setFps(fps)
        camRgb.setInterleaved(False)

        left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
        left.setFps(fps)

        right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
        right.setFps(fps)

        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
        stereo.setLeftRightCheck(True)

        # Linking
        camRgb.video.link(rgbOut.input)
        left.out.link(stereo.left)
        right.out.link(stereo.right)
        stereo.disparity.link(dispOut.input)

        device = dai.Device(pipeline)

        # Create output queues
        queues = {
            "rgb": device.getOutputQueue("rgb", maxSize=4, blocking=False),
            "disp": device.getOutputQueue("disp", maxSize=4, blocking=False),
        }

        return device, queues

    def publish_frames(self):
        rgb_frame = self.queues["rgb"].tryGet()
        disp_frame = self.queues["disp"].tryGet()

        if rgb_frame:
            img = rgb_frame.getCvFrame()
            msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            self.rgb_pub.publish(msg)

        if disp_frame:
            disp = disp_frame.getFrame()
            disp = (disp * (255.0 / disp.max())).astype("uint8")  # Normalize
            msg = self.bridge.cv2_to_imgmsg(disp, encoding="mono8")
            msg.header.stamp = self.get_clock().now().to_msg()
            self.disp_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OakCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
