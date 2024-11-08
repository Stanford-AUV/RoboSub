import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
    Pose2D,
)


class ObjectsDetector(Node):

    def __init__(self):
        super().__init__("objects_detector")

        # Load the YOLO model
        self.model = YOLO("yolo11n.pt")  # Ensure the correct model path

        # Initialize the CvBridge to convert ROS Image messages to OpenCV format
        self.bridge = CvBridge()

        # Create a subscription to the image topic (e.g., /camera/image_raw)
        self.subscription = self.create_subscription(
            Image, "oak/rgb/image_raw", self.image_callback, 10
        )
        self.pub = self.create_publisher(Detection2DArray, "/detections2d", 10)

        # Log to indicate that the node has started
        self.get_logger().info(
            "Objects detector is running and subscribing to image_raw topic"
        )

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Run YOLO model on the image
        results = self.model(cv_image)

        def xywh_to_bbox(xywh_list):
            [[x, y, w, h]] = xywh_list
            # Calculate center coordinates
            center_x = x + w / 2
            center_y = y + h / 2

            # Set size_x and size_y
            size_x = float(w)
            size_y = float(h)

            # Create the Pose2D for the center
            pose = Pose2D()
            pose.position.x = float(center_x)
            pose.position.y = float(center_y)

            return {"center": pose, "size_x": size_x, "size_y": size_y}

        detections_array = Detection2DArray()
        # append detection (i.e. a bounding box) to detections_array
        num_bbox = len(results[0].boxes.xywh)
        for bbox_idx in range(num_bbox):
            detection = Detection2D()

            # init objecthypothsis...
            objhypo = ObjectHypothesisWithPose()
            objhypo.hypothesis.class_id = str(results[0].boxes[bbox_idx].cls)
            objhypo.hypothesis.score = float(results[0].boxes[bbox_idx].conf)
            detection.results.append(objhypo)

            # init 2d boundingbox
            xywh = results[0].boxes[bbox_idx].xywh
            bbox_info = xywh_to_bbox(xywh)
            detection.bbox.center = bbox_info["center"]
            detection.bbox.size_x = bbox_info["size_x"]
            detection.bbox.size_y = bbox_info["size_y"]

            # (opt) init source image

            # append detection to array
            detections_array.detections.append(detection)

        # Annotate the image with detections
        detections_array.header = msg.header
        self.pub.publish(detections_array)

        annotated_frame = results[0].plot()

        # Display the annotated frame (optional)
        cv2.imshow("YOLOv8 Detections", annotated_frame)
        cv2.waitKey(1)  # Wait for a brief moment to update the display

        # (Optional) Save the annotated image as an output file if required
        # cv2.imwrite('output_frame.jpg', annotated_frame)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectsDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
