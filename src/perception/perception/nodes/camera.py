import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import depthai as dai


class Camera(Node):
    def __init__(self, device):
        super().__init__("camera")
        self.get_logger().info("DepthAI camera publisher node has been created!")

        self.device = device

        # ROS2 publisher
        self.bridge = CvBridge()
        self.rgb_pub = self.create_publisher(Image, "oak/rgb/image_raw", 10)
        self.depth_pub = self.create_publisher(Image, "oak/depth/image_raw", 10)

        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        device = self.device
        latestPacket = {}
        latestPacket["rgb"] = None
        latestPacket["disp"] = None

        queueEvents = device.getQueueEvents(("rgb", "disp"))
        for queueName in queueEvents:
            packets = device.getOutputQueue(queueName).tryGetAll()
            if len(packets) > 0:
                latestPacket[queueName] = packets[-1]

        if latestPacket["rgb"] is not None:
            frameRgb = latestPacket["rgb"].getCvFrame()
            self.rgb_pub.publish(self.bridge.cv2_to_imgmsg(frameRgb, "bgr8"))

        if latestPacket["disp"] is not None:
            frameDisp = latestPacket["disp"].getFrame()
            # maxDisparity = stereo.initialConfig.getMaxDisparity()
            # TODO: Move to view cam
            # # Optional, extend range 0..95 -> 0..255, for a better visualisation
            # if 1:
            #     frameDisp = (frameDisp * 255.0 / maxDisparity).astype(np.uint8)
            # # Optional, apply false colorization
            # if 1:
            #     frameDisp = cv2.applyColorMap(frameDisp, cv2.COLORMAP_HOT)
            # frameDisp = np.ascontiguousarray(frameDisp)
            # cv2.imshow(depthWindowName, frameDisp)
            self.depth_pub.publish(
                self.bridge.cv2_to_imgmsg(frameDisp, encoding="passthrough")
            )


def setup_device():
    fps = 30

    # The disparity is computed at this resolution, then upscaled to RGB resolution
    monoResolution = dai.MonoCameraProperties.SensorResolution.THE_720_P

    # Create pipeline
    pipeline = dai.Pipeline()
    device = dai.Device()
    queueNames = []

    # Define sources and outputs
    camRgb = pipeline.create(dai.node.Camera)
    left = pipeline.create(dai.node.MonoCamera)
    right = pipeline.create(dai.node.MonoCamera)
    stereo = pipeline.create(dai.node.StereoDepth)

    rgbOut = pipeline.create(dai.node.XLinkOut)
    disparityOut = pipeline.create(dai.node.XLinkOut)

    rgbOut.setStreamName("rgb")
    queueNames.append("rgb")
    disparityOut.setStreamName("disp")
    queueNames.append("disp")

    # Properties
    rgbCamSocket = dai.CameraBoardSocket.CAM_A

    camRgb.setBoardSocket(rgbCamSocket)
    camRgb.setSize(1280, 720)
    camRgb.setFps(fps)

    # For now, RGB needs fixed focus to properly align with depth.
    # This value was used during calibration
    try:
        calibData = device.readCalibration2()
        lensPosition = calibData.getLensPosition(rgbCamSocket)
        if lensPosition:
            camRgb.initialControl.setManualFocus(lensPosition)
    except:
        raise
    left.setResolution(monoResolution)
    left.setCamera("left")
    left.setFps(fps)
    right.setResolution(monoResolution)
    right.setCamera("right")
    right.setFps(fps)

    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    # LR-check is required for depth alignment
    stereo.setLeftRightCheck(True)
    stereo.setDepthAlign(rgbCamSocket)

    # Linking
    camRgb.video.link(rgbOut.input)
    left.out.link(stereo.left)
    right.out.link(stereo.right)
    stereo.disparity.link(disparityOut.input)

    camRgb.setMeshSource(dai.CameraProperties.WarpMeshSource.CALIBRATION)

    # Connect to device and start pipeline
    return device, pipeline


def main(args=None):
    rclpy.init(args=args)
    device, pipeline = setup_device()
    with device:
        device.startPipeline(pipeline)
        node = Camera(device)
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
