import rclpy
from rclpy import Parameter
from rclpy.node import Node
import cv2
from geometry_msgs.msg import Pose
from vision_msgs.msg import (
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)
import depthai as dai
import numpy as np
import time
import json


class ObjectsLocalizer(Node):

    def __init__(self):
        super().__init__("objects_localizer")

        self.declare_parameter("view_detections", Parameter.Type.BOOL)
        try:
            view_detections = (
                self.get_parameter("view_detections").get_parameter_value().bool_value
            )
        except:
            view_detections = False

        self.pub = self.create_publisher(Detection3DArray, "detections3d", 10)

        # Use https://tools.luxonis.com for generating blob and config files from a PyTorch model (.pt). Set input image size to 416x416.
        nnBlobPath = "yolo11n_openvino_2022.1_6shave.blob"
        configPath = "yolo11n.json"
        syncNN = True

        with open(configPath) as f:
            config = json.load(f)

        nnConfig = config.get("nn_config", {})

        # parse input shape
        if "input_size" in nnConfig:
            W, H = tuple(map(int, nnConfig.get("input_size").split("x")))

        # extract metadata
        metadata = nnConfig.get("NN_specific_metadata", {})
        classes = metadata.get("classes", {})
        coordinates = metadata.get("coordinates", {})
        anchors = metadata.get("anchors", {})
        anchorMasks = metadata.get("anchor_masks", {})
        iouThreshold = metadata.get("iou_threshold", {})
        confidenceThreshold = metadata.get("confidence_threshold", {})

        # parse labels
        nnMappings = config.get("mappings", {})
        labels = nnMappings.get("labels", {})

        # Create pipeline
        pipeline = dai.Pipeline()

        # Define sources and outputs
        camRgb = pipeline.create(dai.node.ColorCamera)
        spatialDetectionNetwork = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
        monoLeft = pipeline.create(dai.node.MonoCamera)
        monoRight = pipeline.create(dai.node.MonoCamera)
        stereo = pipeline.create(dai.node.StereoDepth)
        nnNetworkOut = pipeline.create(dai.node.XLinkOut)

        xoutRgb = pipeline.create(dai.node.XLinkOut)
        xoutNN = pipeline.create(dai.node.XLinkOut)
        xoutDepth = pipeline.create(dai.node.XLinkOut)

        xoutRgb.setStreamName("rgb")
        xoutNN.setStreamName("detections")
        xoutDepth.setStreamName("depth")
        nnNetworkOut.setStreamName("nnNetwork")

        # Properties
        camRgb.setPreviewSize(W, H)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setCamera("left")
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setCamera("right")

        # setting node configs
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        # Align depth map to the perspective of RGB camera, on which inference is done
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        stereo.setOutputSize(
            monoLeft.getResolutionWidth(), monoLeft.getResolutionHeight()
        )
        stereo.setSubpixel(True)

        # Network specific settings
        spatialDetectionNetwork.setConfidenceThreshold(confidenceThreshold)
        spatialDetectionNetwork.setNumClasses(classes)
        spatialDetectionNetwork.setCoordinateSize(coordinates)
        spatialDetectionNetwork.setAnchors(anchors)
        spatialDetectionNetwork.setAnchorMasks(anchorMasks)
        spatialDetectionNetwork.setIouThreshold(iouThreshold)
        spatialDetectionNetwork.setBlobPath(nnBlobPath)
        spatialDetectionNetwork.setNumInferenceThreads(2)
        spatialDetectionNetwork.input.setBlocking(False)

        # Linking
        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)

        camRgb.preview.link(spatialDetectionNetwork.input)
        if syncNN:
            spatialDetectionNetwork.passthrough.link(xoutRgb.input)
        else:
            camRgb.preview.link(xoutRgb.input)

        spatialDetectionNetwork.out.link(xoutNN.input)

        stereo.depth.link(spatialDetectionNetwork.inputDepth)
        spatialDetectionNetwork.passthroughDepth.link(xoutDepth.input)
        spatialDetectionNetwork.outNetwork.link(nnNetworkOut.input)

        # Log to indicate that the node has started
        self.get_logger().info("Objects localizer is running")

        # Connect to device and start pipeline
        with dai.Device(pipeline) as device:
            calibData = device.readCalibration()
            M_rgb = calibData.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, W, H)
            M_rgb_inv = np.linalg.inv(M_rgb)

            self.get_logger().info(str(M_rgb))

            # Output queues will be used to get the rgb frames and nn data from the outputs defined above
            previewQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            detectionNNQueue = device.getOutputQueue(
                name="detections", maxSize=4, blocking=False
            )
            depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
            networkQueue = device.getOutputQueue(
                name="nnNetwork", maxSize=4, blocking=False
            )

            startTime = time.monotonic()
            counter = 0
            fps = 0
            color = (255, 255, 255)
            printOutputLayersOnce = True

            while True:
                inPreview = previewQueue.get()
                inDet = detectionNNQueue.get()
                depth = depthQueue.get()
                inNN = networkQueue.get()

                if printOutputLayersOnce:
                    toPrint = "Output layer names:"
                    for ten in inNN.getAllLayerNames():
                        toPrint = f"{toPrint} {ten},"
                    print(toPrint)
                    printOutputLayersOnce = False

                frame = inPreview.getCvFrame()
                depthFrame = depth.getFrame()  # depthFrame values are in millimeters

                depth_downscaled = depthFrame[::4]
                if np.all(depth_downscaled == 0):
                    min_depth = 0  # Set a default minimum depth value when all elements are zero
                else:
                    min_depth = np.percentile(
                        depth_downscaled[depth_downscaled != 0], 1
                    )
                max_depth = np.percentile(depth_downscaled, 99)
                depthFrameColor = np.interp(
                    depthFrame, (min_depth, max_depth), (0, 255)
                ).astype(np.uint8)
                depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)

                counter += 1
                current_time = time.monotonic()
                if (current_time - startTime) > 1:
                    fps = counter / (current_time - startTime)
                    counter = 0
                    startTime = current_time

                detections = inDet.detections

                # If the frame is available, draw bounding boxes on it and show the frame
                height = frame.shape[0]
                width = frame.shape[1]

                ros_detections = Detection3DArray()
                for detection in detections:
                    roiData = detection.boundingBoxMapping

                    roi = roiData.roi
                    roi = roi.denormalize(
                        depthFrameColor.shape[1], depthFrameColor.shape[0]
                    )
                    topLeft = roi.topLeft()
                    bottomRight = roi.bottomRight()
                    xmin = int(topLeft.x)
                    ymin = int(topLeft.y)
                    xmax = int(bottomRight.x)
                    ymax = int(bottomRight.y)

                    spatialCoordinates = detection.spatialCoordinates

                    label = labels[detection.label]

                    if view_detections:
                        cv2.rectangle(
                            depthFrameColor, (xmin, ymin), (xmax, ymax), color, 1
                        )

                        # Denormalize bounding box
                        x1 = int(detection.xmin * width)
                        x2 = int(detection.xmax * width)
                        y1 = int(detection.ymin * height)
                        y2 = int(detection.ymax * height)

                        cv2.putText(
                            frame,
                            str(label),
                            (x1 + 10, y1 + 20),
                            cv2.FONT_HERSHEY_TRIPLEX,
                            0.5,
                            255,
                        )
                        cv2.putText(
                            frame,
                            "{:.2f}".format(detection.confidence * 100),
                            (x1 + 10, y1 + 35),
                            cv2.FONT_HERSHEY_TRIPLEX,
                            0.5,
                            255,
                        )
                        cv2.putText(
                            frame,
                            f"X: {int(spatialCoordinates.x)} mm",
                            (x1 + 10, y1 + 50),
                            cv2.FONT_HERSHEY_TRIPLEX,
                            0.5,
                            255,
                        )
                        cv2.putText(
                            frame,
                            f"Y: {int(spatialCoordinates.y)} mm",
                            (x1 + 10, y1 + 65),
                            cv2.FONT_HERSHEY_TRIPLEX,
                            0.5,
                            255,
                        )
                        cv2.putText(
                            frame,
                            f"Z: {int(spatialCoordinates.z)} mm",
                            (x1 + 10, y1 + 80),
                            cv2.FONT_HERSHEY_TRIPLEX,
                            0.5,
                            255,
                        )

                        cv2.rectangle(
                            frame, (x1, y1), (x2, y2), color, cv2.FONT_HERSHEY_SIMPLEX
                        )

                    # Bounding box corners in pixel coordinates
                    top_left = np.array([xmin, ymin, 1])
                    bottom_right = np.array([xmax, ymax, 1])

                    # Depth of the detected object
                    z = spatialCoordinates.z  # Depth in mm

                    # Transform corners to world coordinates
                    world_top_left = z * M_rgb_inv @ top_left
                    world_bottom_right = z * M_rgb_inv @ bottom_right

                    # Calculate world dimensions
                    world_width = world_bottom_right[0] - world_top_left[0]
                    world_height = world_bottom_right[1] - world_top_left[1]

                    ros_detection = Detection3D()
                    ros_objhypo = ObjectHypothesisWithPose()
                    ros_objhypo.hypothesis.class_id = label
                    ros_objhypo.hypothesis.score = detection.confidence
                    ros_detection.results.append(ros_objhypo)

                    ros_center = Pose()
                    ros_center.position.x = spatialCoordinates.x / 1000.0  # in m
                    ros_center.position.y = spatialCoordinates.y / 1000.0  # in m
                    ros_center.position.z = spatialCoordinates.z / 1000.0  # in m

                    ros_detection.bbox.center = ros_center
                    ros_detection.bbox.size.x = world_width / 1000.0  # in m
                    ros_detection.bbox.size.y = world_height / 1000.0  # in m
                    ros_detection.bbox.size.z = 0.0

                    ros_detections.detections.append(ros_detection)

                self.pub.publish(ros_detections)

                if view_detections:
                    cv2.putText(
                        frame,
                        "NN fps: {:.2f}".format(fps),
                        (2, frame.shape[0] - 4),
                        cv2.FONT_HERSHEY_TRIPLEX,
                        0.4,
                        color,
                    )
                    cv2.imshow("depth", depthFrameColor)
                    cv2.imshow("rgb", frame)

                    if cv2.waitKey(1) == ord("q"):
                        break


def main(args=None):
    rclpy.init(args=args)
    node = ObjectsLocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
