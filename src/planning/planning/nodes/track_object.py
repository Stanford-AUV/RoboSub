#!/usr/bin/env python3
import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from vision_msgs.msg import Detection3DArray
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

# ⬇️ use your package name if not 'msgs'
from msgs.msg import Waypoint

import tf2_ros
import tf_transformations as tft  # provided by geometry2 (tf_transformations)

def quat_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    qx, qy, qz, qw = tft.quaternion_from_euler(roll, pitch, yaw)
    q = Quaternion()
    q.x, q.y, q.z, q.w = qx, qy, qz, qw
    return q

def pose_to_tuple(p: Pose) -> Tuple[float, float, float]:
    return (p.position.x, p.position.y, p.position.z)

def normalize(vx, vy, vz) -> Tuple[float, float, float]:
    n = math.sqrt(vx*vx + vy*vy + vz*vz)
    if n < 1e-6:
        return 0.0, 0.0, 0.0
    return vx/n, vy/n, vz/n

class ObjectFollowerWaypoints(Node):
    """
    Subscribes:
      - detections_topic (vision_msgs/Detection3DArray): 3D detections in *camera optical frame*
      - /odometry/filtered (nav_msgs/Odometry): robot pose in target_frame

    Publishes:
      - /waypoint (msgs/Waypoint): desired odometry target + meta
    """

    def __init__(self):
        super().__init__("object_follower_waypoints")

        # --- Parameters ---
        self.declare_parameter("detections_topic", "/detections3d/front")
        self.declare_parameter("camera_source", "front")  # "front" or "bottom"
        self.declare_parameter("front_camera_frame", "camera_front_optical_frame")
        self.declare_parameter("bottom_camera_frame", "camera_bottom_optical_frame")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("target_frame", "odom")
        self.declare_parameter("waypoint_topic", "waypoint")

        self.declare_parameter("update_hz", 2.0)
        self.declare_parameter("min_score", 0.30)
        self.declare_parameter("select_class", "")
        self.declare_parameter("standoff_m", 0.8)
        self.declare_parameter("lateral_offset_m", 0.0)
        self.declare_parameter("vertical_offset_m", 0.0)
        self.declare_parameter("object_offset_cam", [0.0, 0.0, 0.0])
        self.declare_parameter("lock_roll", True)
        self.declare_parameter("enable_orientation", True)

        # New: metadata for Waypoint msg
        self.declare_parameter("waypoint_purpose", "object_follow")
        self.declare_parameter("waypoint_subject", "")
        self.declare_parameter("hold_time", 0.0)

        # Fetch params
        self.detections_topic = self.get_parameter("detections_topic").get_parameter_value().string_value
        self.camera_source = self.get_parameter("camera_source").get_parameter_value().string_value.lower()
        self.front_cam_frame = self.get_parameter("front_camera_frame").get_parameter_value().string_value
        self.bottom_cam_frame = self.get_parameter("bottom_camera_frame").get_parameter_value().string_value
        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self.target_frame = self.get_parameter("target_frame").get_parameter_value().string_value
        self.waypoint_topic = self.get_parameter("waypoint_topic").get_parameter_value().string_value

        self.update_hz = float(self.get_parameter("update_hz").value)
        self.min_score = float(self.get_parameter("min_score").value)
        self.select_class = self.get_parameter("select_class").get_parameter_value().string_value or None
        self.standoff_m = float(self.get_parameter("standoff_m").value)
        self.lateral_offset_m = float(self.get_parameter("lateral_offset_m").value)
        self.vertical_offset_m = float(self.get_parameter("vertical_offset_m").value)
        self.object_offset_cam = list(self.get_parameter("object_offset_cam").value)
        self.lock_roll = bool(self.get_parameter("lock_roll").value)
        self.enable_orientation = bool(self.get_parameter("enable_orientation").value)

        self.wp_purpose = self.get_parameter("waypoint_purpose").get_parameter_value().string_value
        self.wp_subject_param = self.get_parameter("waypoint_subject").get_parameter_value().string_value
        self.hold_time = float(self.get_parameter("hold_time").value)

        if self.camera_source not in ("front", "bottom"):
            self.get_logger().warn("camera_source must be 'front' or 'bottom'; defaulting to 'front'")
            self.camera_source = "front"

        self.camera_frame = self.front_cam_frame if self.camera_source == "front" else self.bottom_cam_frame

        # TF buffer & listener
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriptions / Publications
        self.sub_det = self.create_subscription(Detection3DArray, self.detections_topic, self.on_detections, 10)
        self.sub_odom = self.create_subscription(Odometry, "/odometry/filtered", self.on_odom, 10)

        # Matching torpedo_task QoS (Transient Local + Reliable)
        wp_qos = QoSProfile(depth=1)
        wp_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        wp_qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.pub_wp = self.create_publisher(Waypoint, self.waypoint_topic, wp_qos)

        # Timer for publishing
        period = 1.0 / max(self.update_hz, 1e-3)
        self.timer = self.create_timer(period, self.on_timer)

        # State
        self.last_detection_in_cam: Optional[Pose] = None
        self.last_detection_time = None
        self.latest_odom: Optional[Odometry] = None
        self.last_label: Optional[str] = None

        self.get_logger().info(
            f"ObjectFollowerWaypoints (Waypoint publisher) active.\n"
            f"  detections: {self.detections_topic} (frame: {self.camera_frame})\n"
            f"  odom: /odometry/filtered (base: {self.base_frame}, target: {self.target_frame})\n"
            f"  publish: {self.waypoint_topic} @ {self.update_hz} Hz\n"
            f"  purpose/subject: {self.wp_purpose!r}/{self.wp_subject_param or '(auto from detection)'}\n"
            f"  standoff: {self.standoff_m:.2f} m, offsets (base Y,Z): "
            f"{self.lateral_offset_m:.2f}, {self.vertical_offset_m:.2f} m\n"
            f"  object_offset_cam: {self.object_offset_cam}"
        )

    # --- Callbacks ---

    def on_odom(self, msg: Odometry):
        self.latest_odom = msg

    def on_detections(self, msg: Detection3DArray):
        best_pose = None
        best_score = -1.0
        best_label = None

        for det in msg.detections:
            score = det.results[0].score if det.results else 1.0
            if score < self.min_score:
                continue

            # optional class filter
            det_label = None
            if det.results:
                # prefer class_name if available
                det_label = getattr(det.results[0].hypothesis, "class_name", "") or \
                            getattr(det.results[0].hypothesis, "class_id", "")
            if self.select_class:
                if not det_label or str(det_label) != str(self.select_class):
                    continue

            p = det.results[0].pose.pose if det.results else det.bbox.center
            if score > best_score:
                best_score = score
                best_label = str(det_label) if det_label is not None else None
                best_pose = Pose()
                best_pose.position = p.position
                best_pose.orientation = p.orientation

        if best_pose is None:
            return

        # apply camera->object bias
        best_pose.position.x += float(self.object_offset_cam[0])
        best_pose.position.y += float(self.object_offset_cam[1])
        best_pose.position.z += float(self.object_offset_cam[2])

        self.last_detection_in_cam = best_pose
        self.last_detection_time = msg.header.stamp
        self.last_label = best_label

    # --- Core timer: compute and publish waypoint ---

    def on_timer(self):
        if self.last_detection_in_cam is None or self.latest_odom is None:
            return

        # 1) cam -> target_frame
        obj_in_target = self.transform_pose(self.last_detection_in_cam, self.camera_frame, self.target_frame)
        if obj_in_target is None:
            return

        # 2) base in target
        base_in_target = self.lookup_transform_pose(self.target_frame, self.base_frame)
        if base_in_target is None:
            return

        bx, by, bz = pose_to_tuple(base_in_target)
        ox, oy, oz = pose_to_tuple(obj_in_target)

        # 3) LOS
        dirx, diry, dirz = normalize(ox - bx, oy - by, oz - bz)
        if (dirx, diry, dirz) == (0.0, 0.0, 0.0):
            return

        # 4) standoff
        wp_x = ox - self.standoff_m * dirx
        wp_y = oy - self.standoff_m * diry
        wp_z = oz - self.standoff_m * dirz

        # 5) base-frame offsets -> target_frame
        off_in_target = self.vector_base_to_target(0.0, self.lateral_offset_m, self.vertical_offset_m)
        if off_in_target is not None:
            wp_x += off_in_target[0]
            wp_y += off_in_target[1]
            wp_z += off_in_target[2]

        # 6) orientation
        if self.enable_orientation:
            look_vec_in_base = self.vector_target_to_base(ox - bx, oy - by, oz - bz)
            if look_vec_in_base is None:
                return
            lx, ly, lz = look_vec_in_base
            yaw = math.atan2(ly, lx)
            hyp = math.sqrt(lx*lx + ly*ly)
            pitch = math.atan2(-lz, max(hyp, 1e-9))
            roll = 0.0 if self.lock_roll else 0.0
            q = quat_from_rpy(roll, pitch, yaw)
        else:
            q = self.latest_odom.pose.pose.orientation

        # 7) Build Waypoint (with Odometry target)
        now = self.get_clock().now().to_msg()

        odom_target = Odometry()
        odom_target.header.stamp = now
        odom_target.header.frame_id = self.target_frame   # target pose is expressed in this frame
        odom_target.child_frame_id = self.base_frame      # optional; who should achieve this pose

        odom_target.pose.pose.position = Point(x=wp_x, y=wp_y, z=wp_z)
        odom_target.pose.pose.orientation = q
        # twist left at 0 (desired stop); covariances left default

        wp_msg = Waypoint()
        wp_msg.purpose = self.wp_purpose
        # prefer explicit param; fall back to last detection label if available
        wp_msg.subject = self.wp_subject_param or (self.last_label or "")
        wp_msg.target = odom_target
        wp_msg.hold_time = float(self.hold_time)

        self.pub_wp.publish(wp_msg)

    # --- TF helpers (unchanged) ---

    def transform_pose(self, pose_in: Pose, from_frame: str, to_frame: str) -> Optional[Pose]:
        try:
            tf = self.tf_buffer.lookup_transform(to_frame, from_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().throttle_warn(self.get_clock(), 2000, f"TF {from_frame}->{to_frame} unavailable: {e}")
            return None

        try:
            T = tft.concatenate_matrices(
                tft.translation_matrix([tf.transform.translation.x,
                                        tf.transform.translation.y,
                                        tf.transform.translation.z]),
                tft.quaternion_matrix([tf.transform.rotation.x,
                                       tf.transform.rotation.y,
                                       tf.transform.rotation.z,
                                       tf.transform.rotation.w])
            )
            p = [pose_in.position.x, pose_in.position.y, pose_in.position.z, 1.0]
            pout = T @ p
            out = Pose()
            out.position.x, out.position.y, out.position.z = pout[0], pout[1], pout[2]

            q_tf = [tf.transform.rotation.x, tf.transform.rotation.y,
                    tf.transform.rotation.z, tf.transform.rotation.w]
            q_in = [pose_in.orientation.x, pose_in.orientation.y,
                    pose_in.orientation.z, pose_in.orientation.w]
            q_out = tft.quaternion_multiply(q_tf, q_in)
            out.orientation = Quaternion(x=q_out[0], y=q_out[1], z=q_out[2], w=q_out[3])
            return out
        except Exception as e:
            self.get_logger().warn(f"Pose transform failed: {e}")
            return None

    def lookup_transform_pose(self, to_frame: str, from_frame: str) -> Optional[Pose]:
        try:
            tf = self.tf_buffer.lookup_transform(to_frame, from_frame, rclpy.time.Time())
            out = Pose()
            out.position.x = tf.transform.translation.x
            out.position.y = tf.transform.translation.y
            out.position.z = tf.transform.translation.z
            out.orientation = tf.transform.rotation
            return out
        except Exception as e:
            self.get_logger().throttle_warn(self.get_clock(), 2000, f"TF {from_frame}->{to_frame} unavailable: {e}")
            return None

    def vector_base_to_target(self, vx_b, vy_b, vz_b) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(self.target_frame, self.base_frame, rclpy.time.Time())
            q = [tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w]
            R = tft.quaternion_matrix(q)[:3, :3]
            v = R @ [vx_b, vy_b, vz_b]
            return float(v[0]), float(v[1]), float(v[2])
        except Exception as e:
            self.get_logger().throttle_warn(self.get_clock(), 2000, f"TF rot base->target unavailable: {e}")
            return None

    def vector_target_to_base(self, vx_t, vy_t, vz_t) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.target_frame, rclpy.time.Time())
            q = [tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w]
            R = tft.quaternion_matrix(q)[:3, :3]
            v = R @ [vx_t, vy_t, vz_t]
            return float(v[0]), float(v[1]), float(v[2])
        except Exception as e:
            self.get_logger().throttle_warn(self.get_clock(), 2000, f"TF rot target->base unavailable: {e}")
            return None

def main():
    rclpy.init()
    node = ObjectFollowerWaypoints()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
