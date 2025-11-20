import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
from ultralytics import YOLO
from cv_bridge import CvBridge
import numpy as np
import cv2

def compute_gazebo_intrinsics(horizontal_fov, width, height):
    """
    Computes fx, fy, cx, cy from Gazebo camera parameters.
    horizontal_fov: radians
    width, height: image resolution
    """

    # focal length from pinhole model
    fx = width / (2 * np.tan(horizontal_fov / 2.0))
    fy = fx  # Gazebo has square pixels

    # principal point at image center
    cx = width / 2.0
    cy = height / 2.0

    return fx, fy, cx, cy

class DetectionNode3D(Node):
    def __init__(self):
        super().__init__("detection_node_3d")

        # Topics as parameters
        self.declare_parameter("rgb_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")

        rgb_topic = self.get_parameter("rgb_topic").get_parameter_value().string_value
        depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value

        # Subscriptions
        self.rgb_sub = self.create_subscription(Image, rgb_topic, self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(Image, depth_topic, self.depth_callback, 10)

        self.fx, self.fy, self.cx, self.cy = compute_gazebo_intrinsics(
            horizontal_fov=1.047, width=416, height=416
        )
        # Publishers
        self.image_pub = self.create_publisher(Image, 'gz/detection_image', 10)
        self.data_pub = self.create_publisher(Detection3DArray, '/detection_data_3d', 10)

        # YOLO model
        self.model = YOLO("yolo11n.pt")
        self.bridge = CvBridge()
        self.names = self.model.names

        # Store latest depth
        self.latest_depth = None

    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def rgb_callback(self, msg):
        if self.latest_depth is None:
            self.get_logger().warn("No depth data yet")
            return
        
        cv_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(cv_rgb, conf=0.4)[0]

        # Convert YOLO 2D detections -> 3D
        det_array_3d = Detection3DArray()
        det_array_3d.header.stamp = self.get_clock().now().to_msg()
        det_array_3d.header.frame_id = "camera_frame"

        for box in results.boxes.data:
            x1, y1, x2, y2, conf, cls = box.cpu().numpy()
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            h, w = self.latest_depth.shape
            cx = np.clip(cx, 0, w-1)
            cy = np.clip(cy, 0, h-1)
            # Get depth at center pixel
            z = float(self.latest_depth[cy, cx])
            if not np.isfinite(z) or z <= 0.0:
                continue

            # Project to camera frame
            X = (cx - self.cx) * z / self.fx
            Y = (cy - self.cy) * z / self.fy

            # Detection3D
            det = Detection3D()
            det.header.stamp = self.get_clock().now().to_msg()
            det.header.frame_id = "camera_frame"

            # Object hypothesis
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = self.names[int(cls)]
            hyp.hypothesis.score = float(conf)
            hyp.pose.pose.position.x = X
            hyp.pose.pose.position.y = Y
            hyp.pose.pose.position.z = z
            hyp.pose.covariance = [0.0]*36

            if hyp.hypothesis.class_id != "tv":
                continue
            det.results.append(hyp)
            det_array_3d.detections.append(det)

            # Draw 3D point on RGB image
            cv2.circle(cv_rgb, (cx, cy), 5, (0,0,255), -1)
            label = f"{hyp.hypothesis.class_id}: {z:.2f}m Confidence: {conf:.2f}"
            cv2.putText(cv_rgb, label, (cx+5, cy-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 1, cv2.LINE_AA)

        self.data_pub.publish(det_array_3d)

        # Publish annotated RGB image
        annotated_msg = self.bridge.cv2_to_imgmsg(cv_rgb, encoding='bgr8')
        self.image_pub.publish(annotated_msg)
        # self.get_logger().info("Published 3D detection image and data")

def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode3D()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
