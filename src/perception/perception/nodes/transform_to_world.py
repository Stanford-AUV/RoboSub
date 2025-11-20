import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
import numpy as np
import tf_transformations as tf

class DetectionTransformer(Node):
    def __init__(self):
        super().__init__('detection_transformer')

        # Camera-to-robot transform
        t = [0.0, 0.0, 0.0]  # translation (modify if needed)
        R_mat = np.array([[0, 0, 1],
                          [-1, 0, 0],
                          [0, -1, 0]])

        # Convert rotation matrix + translation to 4x4 homogeneous transform
        T_cam_robot = np.eye(4)
        T_cam_robot[:3, :3] = R_mat
        T_cam_robot[:3, 3] = t
        self.T_cam_robot = T_cam_robot

        # Latest robot-to-world transform
        self.T_robot_world = np.eye(4)

        # Subscribers
        self.create_subscription(Odometry, "/odometry/filtered", self.odom_callback, 10)
        self.create_subscription(Detection3DArray, "/detection_data_3d", self.detections_callback, 10)

        # Publisher
        self.pub = self.create_publisher(Detection3DArray, "/detections_world", 10)

    def odom_callback(self, msg: Odometry):
        pos = [msg.pose.pose.position.x,
               msg.pose.pose.position.y,
               msg.pose.pose.position.z]
        quat = [msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w]

        # 4x4 homogeneous transform
        T = tf.quaternion_matrix(quat)
        T[:3, 3] = pos
        self.T_robot_world = T

    def detections_callback(self, msg: Detection3DArray):
        transformed_msg = Detection3DArray()
        transformed_msg.header = msg.header

        for detection in msg.detections:
            det_world = Detection3D()
            det_world.header = detection.header
            det_world.results = []

            res = detection.results[0]
            obj_world = ObjectHypothesisWithPose()
            obj_world.hypothesis = res.hypothesis

            # Position as homogeneous vector
            p = [res.pose.pose.position.x,
                 res.pose.pose.position.y,
                 res.pose.pose.position.z,
                 1.0]

            # Transform from camera -> robot -> world
            p_world = self.T_robot_world @ self.T_cam_robot @ p

            if not np.all(np.isfinite(p_world[:3])):
                self.get_logger().warn("Skipping detection with invalid coordinates")
                continue

            obj_world.pose.pose.position.x = p_world[0]
            obj_world.pose.pose.position.y = p_world[1]
            obj_world.pose.pose.position.z = p_world[2]

            # Transform orientation if you want (optional, here we keep original)
            obj_world.pose.pose.orientation = res.pose.pose.orientation

            det_world.results.append(obj_world)
            transformed_msg.detections.append(det_world)

            # Log detection in world frame
            self.get_logger().info(
                f"Detection '{res.hypothesis.class_id}' -> world-frame: "
                f"x={p_world[0]:.3f}, y={p_world[1]:.3f}, z={p_world[2]:.3f}, "
                f"score={res.hypothesis.score:.2f}"
            )

        # Publish all transformed detections
        self.pub.publish(transformed_msg)
        self.get_logger().info(f"Published {len(transformed_msg.detections)} world-frame detections")

def main(args=None):
    rclpy.init(args=args)
    node = DetectionTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
