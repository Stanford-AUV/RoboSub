import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from ultralytics import YOLO
from cv_bridge import CvBridge

class DetectionNode(Node):
    def __init__(self):
        super().__init__("detection_node")

        self.camera_sub =self.create_subscription(Image, 'gz/camera', self.image_callback, 10)

        self.image_pub = self.create_publisher(Image, 'gz/detection_image', 10)
        self.data_pub = self.create_publisher(Detection2DArray, 'detection_data', 10)
        self.model = YOLO("yolo11n.pt")
        self.bridge = CvBridge()
        self.names = self.model.names

    def image_callback(self, msg):
        self.get_logger().info("Received image")

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(cv_image)[0]

        self.publish_detection_image(results.plot())
        self.publish_detection_data(results) # shape: [num_boxes, 6] -> [x1, y1, x2, y2, confidence, class_id]

    def publish_detection_image(self, annotated_frame):
        detection_image = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')     
        self.image_pub.publish(detection_image)
        self.get_logger().info(f"Published detection image to 'gz/detection_image'")

    def publish_detection_data(self, results, frame_id="camera_frame"):
        det_array = Detection2DArray()
        det_array.header.stamp = self.get_clock().now().to_msg()
        det_array.header.frame_id = frame_id

        boxes = results.boxes.data
        for box in boxes:
            x1, y1, x2, y2, conf, cls = box.cpu().numpy()
            det = Detection2D()
            det.header.stamp = self.get_clock().now().to_msg()
            det.header.frame_id = frame_id

            # center and size
            det.bbox.center.position.x = float((x1 + x2) / 2)
            det.bbox.center.position.y = float((y1 + y2) / 2)
            det.bbox.size_x = float(x2 - x1)
            det.bbox.size_y = float(y2 - y1)

            # object hypothesis
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = self.names[int(cls)]
            hyp.hypothesis.score = float(conf)

            hyp.pose.pose.position.x = float((x1 + x2) / 2)
            hyp.pose.pose.position.y = float((y1 + y2) / 2)
            hyp.pose.pose.position.z = 0.0  # Assuming 2D detection
            hyp.pose.covariance = [0.0] * 36  # Placeholder covariance

            det.results.append(hyp)

            det_array.detections.append(det)

        self.data_pub.publish(det_array)

def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()