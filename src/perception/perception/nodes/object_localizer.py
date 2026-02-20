#!/usr/bin/env python3
"""Camera-agnostic object localizer: subscribes to AlignedDepthImage, runs YOLO + depth lookup + deprojection."""

import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from vision_msgs.msg import (
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)
from msgs.msg import AlignedDepthImage
from ultralytics import YOLO


class ObjectLocalizer(Node):
    def __init__(self):
        super().__init__("object_localizer")

        self.declare_parameter("model_name", "yolo-Weights/yolov8n.pt")
        self.declare_parameter("object_id", "person")
        self.declare_parameter("camera_key", "oak_0")
        self.declare_parameter("aligned_topic", "")

        self._model_name = self.get_parameter("model_name").get_parameter_value().string_value
        self._object_id = self.get_parameter("object_id").get_parameter_value().string_value
        camera_key = self.get_parameter("camera_key").get_parameter_value().string_value
        aligned_topic = self.get_parameter("aligned_topic").get_parameter_value().string_value
        if not aligned_topic:
            aligned_topic = f"/camera/{camera_key}/aligned"

        self._model = YOLO(self._model_name)
        self._bridge = CvBridge()

        self._sub = self.create_subscription(
            AlignedDepthImage,
            aligned_topic,
            self._callback,
            10,
        )
        self._pub = self.create_publisher(Detection3DArray, "detections3d", 10)
        self.get_logger().info(f"Subscribed to {aligned_topic}, publishing detections3d")

    def _callback(self, msg: AlignedDepthImage):
        try:
            rgb = self._bridge.imgmsg_to_cv2(msg.rgb, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error decoding RGB: {e}")
            return
        try:
            depth = self._bridge.imgmsg_to_cv2(msg.depth, "passthrough")
        except Exception as e:
            self.get_logger().error(f"Error decoding depth: {e}")
            return

        if depth.dtype != np.uint16:
            depth = np.asarray(depth, dtype=np.uint16)

        k = np.array(msg.camera_info.k).reshape(3, 3)
        fx, fy = k[0, 0], k[1, 1]
        cx, cy = k[0, 2], k[1, 2]
        frame_id = msg.camera_info.header.frame_id
        stamp = msg.rgb.header.stamp

        try:
            results = self._model(rgb, stream=True)
        except Exception as e:
            self.get_logger().error(f"YOLO error: {e}")
            return

        detections_3d = Detection3DArray()
        detections_3d.header.stamp = stamp
        detections_3d.header.frame_id = frame_id

        h, w = rgb.shape[:2]
        for res in results:
            boxes = res.boxes.xyxy.cpu().numpy()
            scores = res.boxes.conf.cpu().numpy()
            classes = res.boxes.cls.cpu().numpy()

            for box, score, cls in zip(boxes, scores, classes):
                class_name = self._model.names[int(cls)]
                if class_name != self._object_id:
                    continue

                x1, y1, x2, y2 = box
                cx_px = (x1 + x2) / 2.0
                cy_px = (y1 + y2) / 2.0

                u, v = int(round(cx_px)), int(round(cy_px))
                if not (0 <= u < depth.shape[1] and 0 <= v < depth.shape[0]):
                    continue
                z_mm = float(depth[v, u])
                if z_mm <= 0:
                    continue

                z_m = z_mm / 1000.0
                x_cam = (cx_px - cx) * z_m / fx
                y_cam = (cy_px - cy) * z_m / fy

                world_top_left = np.array([x1, y1, 1.0])
                world_bottom_right = np.array([x2, y2, 1.0])
                k_inv = np.linalg.inv(k)
                ptl = z_m * (k_inv @ world_top_left)
                pbr = z_m * (k_inv @ world_bottom_right)
                size_x = abs(pbr[0] - ptl[0])
                size_y = abs(pbr[1] - ptl[1])

                detection = Detection3D()
                detection.header = detections_3d.header
                hypo = ObjectHypothesisWithPose()
                hypo.hypothesis.class_id = class_name
                hypo.hypothesis.score = float(score)
                hypo.pose.pose.orientation.w = 1.0
                detection.results.append(hypo)

                center = Pose()
                center.position.x = x_cam
                center.position.y = y_cam
                center.position.z = z_m
                detection.bbox.center = center
                detection.bbox.size.x = size_x
                detection.bbox.size.y = size_y
                detection.bbox.size.z = 0.0

                detections_3d.detections.append(detection)

        self._pub.publish(detections_3d)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectLocalizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
