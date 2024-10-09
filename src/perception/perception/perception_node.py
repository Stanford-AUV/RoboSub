from typing import List, Dict

import rclpy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.lifecycle import LifecycleState

from cv_bridge import CvBridge

from torch import cuda
from ultralytics import YOLO
from ultralytics.engine.results import Results
from ultralytics.engine.results import Boxes

from std_srvs.srv import SetBool
from sensor_msgs.msg import Image
from yolov8_msgs.msg import Point2D
from yolov8_msgs.msg import BoundingBox2D
from yolov8_msgs.msg import Detection
from yolov8_msgs.msg import DetectionArray


class Yolov8Node(LifecycleNode):

    def __init__(self) -> None:
        super().__init__("yolov8_node")

        # params
        self.declare_parameter("model", "./best.pt")
        self.declare_parameter("device", "cuda")
        self.declare_parameter("threshold", 0.5)
        self.declare_parameter("enable", True)
        self.declare_parameter("image_reliability",
                               QoSReliabilityPolicy.BEST_EFFORT)

        self.get_logger().info("Yolov8 Node created")

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Configuring {self.get_name()}")


        self.model = self.get_parameter(
            "model").get_parameter_value().string_value

        self.device = self.get_parameter(
            "device").get_parameter_value().string_value

        self.threshold = self.get_parameter(
            "threshold").get_parameter_value().double_value

        self.enable = self.get_parameter(
            "enable").get_parameter_value().bool_value

        self.reliability = self.get_parameter(
            "image_reliability").get_parameter_value().integer_value

        self.image_qos_profile = QoSProfile(
            reliability=self.reliability,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self._pub = self.create_lifecycle_publisher(
            DetectionArray, "detections", 10)
        self._srv = self.create_service(
            SetBool, "enable", self.enable_cb
        )
        self.cv_bridge = CvBridge()

        print("Using GPU:")
        import torch
        print(torch.cuda.is_available())
        print("################")
        return TransitionCallbackReturn.SUCCESS

    def enable_cb(self, request, response):
        self.enable = request.data
        response.success = True
        return response

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Activating {self.get_name()}")

        self.yolo = YOLO(self.model)

        if "v10" not in self.model:
            self.yolo.fuse()

        # subs
        self._sub = self.create_subscription(
            Image,
            "oak/rgb/image_raw",
            self.image_cb,
            self.image_qos_profile
        )

        super().on_activate(state)

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Deactivating {self.get_name()}")

        del self.yolo
        if "cuda" in self.device:
            self.get_logger().info("Clearing CUDA cache")
            cuda.empty_cache()

        self.destroy_subscription(self._sub)
        self._sub = None

        super().on_deactivate(state)

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Cleaning up {self.get_name()}")

        self.destroy_publisher(self._pub)

        del self.image_qos_profile

        return TransitionCallbackReturn.SUCCESS

    def parse_hypothesis(self, results: Results) -> List[Dict]:

        hypothesis_list = []

        box_data: Boxes
        for box_data in results.boxes:
            hypothesis = {
                "class_id": int(box_data.cls),
                "class_name": self.yolo.names[int(box_data.cls)],
                "score": float(box_data.conf)
            }
            hypothesis_list.append(hypothesis)

        return hypothesis_list

    def parse_boxes(self, results: Results) -> List[BoundingBox2D]:

        boxes_list = []

        box_data: Boxes
        for box_data in results.boxes:

            msg = BoundingBox2D()

            # get boxes values
            box = box_data.xywh[0]
            msg.center.position.x = float(box[0])
            msg.center.position.y = float(box[1])
            msg.size.x = float(box[2])
            msg.size.y = float(box[3])

            # append msg
            boxes_list.append(msg)

        return boxes_list

    def image_cb(self, msg: Image) -> None:

        if self.enable:

            # convert image + predict
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg)
            results = self.yolo.predict(
                source=cv_image,
                verbose=False,
                stream=False,
                conf=self.threshold,
                device=self.device
            )
            results: Results = results[0].cpu()

            if results.boxes:
                hypothesis = self.parse_hypothesis(results)
                boxes = self.parse_boxes(results)
                print("boxes:", boxes)

            else:
                print("empty")
            # create detection msgs
            detections_msg = DetectionArray()

            for i in range(len(results)):

                aux_msg = Detection()

                if results.boxes:
                    aux_msg.class_id = hypothesis[i]["class_id"]
                    aux_msg.class_name = hypothesis[i]["class_name"]
                    aux_msg.score = hypothesis[i]["score"]

                    aux_msg.bbox = boxes[i]

                detections_msg.detections.append(aux_msg)

            # publish detections
            detections_msg.header = msg.header
            self._pub.publish(detections_msg)

            del results
            del cv_image


def main():
    rclpy.init()
    node = Yolov8Node()
    node.trigger_configure()
    node.trigger_activate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

