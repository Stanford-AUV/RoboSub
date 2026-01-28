import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesis
from std_msgs.msg import Header
from ultralytics import YOLO


class ObjectDetections(Node):
    def __init__(self, model_name, object_id, camera_key):
        super().__init__(f"{camera_key}_{object_id}_detections")

        self.model = YOLO(model_name)
        self.object_id = object_id
        self.camera_key = camera_key

        self.bridge = CvBridge()

        rgb_topic = f"/camera/{self.camera_key}/rgb"
        box_topic = f"/{self.object_id}/{self.camera_key}/box_coords"

        self.camera_rgb = self.create_subscription(
            Image, rgb_topic, self.image_callback, 10
        )

        self.bounding_boxes = self.create_publisher(Detection2DArray, box_topic, 10)

        self.get_logger().info(f"Subscribed to {rgb_topic}")

    def image_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            results = self.model(img, stream=True)
            self.bounding_boxes.publish(self.yolo_to_detection2darray(results=results))

        except Exception as e:
            self.get_logger().error(f"Error decoding rgb image from {self.camera_key}")

    def yolo_to_detection2darray(self, results, frame_id="camera_frame"):
        """
        Convert YOLO results to vision_msgs/Detection2DArray
        """
        detection_array = Detection2DArray()
        detection_array.header = Header()
        detection_array.header.stamp = self.get_clock().now()
        detection_array.header.frame_id = frame_id

        for res in results:  # if using stream=True, this iterates over frames
            # res.boxes.xyxy shape: (n,4)
            boxes = res.boxes.xyxy.cpu().numpy()  # numpy array [x1,y1,x2,y2]
            scores = res.boxes.conf.cpu().numpy()
            classes = res.boxes.cls.cpu().numpy()

            for box, score, cls in zip(boxes, scores, classes):
                x1, y1, x2, y2 = box
                width = x2 - x1
                height = y2 - y1
                cx = x1 + width / 2
                cy = y1 + height / 2

                detection = Detection2D()
                detection.header = detection_array.header
                detection.bbox.center.x = cx
                detection.bbox.center.y = cy
                detection.bbox.size_x = width
                detection.bbox.size_y = height

                # ObjectHypothesis contains the class id and score
                hypothesis = ObjectHypothesis()
                hypothesis.id = int(cls)
                hypothesis.score = float(score)
                detection.results.append(hypothesis)

                name = self.model.names[hypothesis.id]
                if name == self.object_id:
                    detection_array.detections.append(detection)

        return detection_array


def main(args=None):
    rclpy.init(args=args)

    node = ObjectDetections("yolo-Weights/yolov8n.pt", "person", "oak_0")

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
