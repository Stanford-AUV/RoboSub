"""
gate_detector.py
----------------
Perception node for the RoboSub gate task.

Detects the gate using its distinctive BLACK/RED color panels from an
RGB+Depth camera (RealSense or OAK), then publishes:
  /perception/gate/pose  (geometry_msgs/PoseStamped) — 3-D gate center pose
  /perception/gate/side  (std_msgs/String)            — "survey_repair" or
                                                        "search_rescue"

Gate description:
  - Two vertical PVC legs wrapped in 3-inch corrugated plastic panels
  - Right leg : RED top, BLACK bottom
  - Left leg  : BLACK top, RED bottom
  - A RED horizontal center-divider splits the gate in half vertically
  - Each half bears a symbol (compass/hammer vs life-ring/SOS)

Detection strategy:
  1. Segment red and black regions in HSV.
  2. Find the two largest vertical red+black blobs → gate legs.
  3. Identify the center divider (tall thin horizontal red band between legs).
  4. Classify left/right symbol using ORB feature matching against stored
     templates (falls back to color-ratio heuristic if templates absent).
  5. Deproject gate bounding box centre through the aligned depth frame to
     get a metric 3-D position; orientation aligned to camera +Z forward.

Coordinate convention (published PoseStamped, frame_id = "map"):
  - Position  : centre of the gate opening in world coordinates
  - Orientation: quaternion whose +X axis points *through* the gate
                 (used by mission_executor to build approach/exit waypoints)
"""

import os
import math

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

# ---------------------------------------------------------------------------
# HSV colour ranges  (H in [0,180], S/V in [0,255] for OpenCV)
# ---------------------------------------------------------------------------
RED_RANGES = [
    ((0,   120, 60), (10,  255, 255)),   # lower red hue wrap
    ((165, 120, 60), (180, 255, 255)),   # upper red hue wrap
]
BLACK_RANGE = ((0, 0, 0), (180, 255, 60))

# Minimum contour area (px²) to consider as a gate leg panel
MIN_PANEL_AREA = 1500

# How many frames a detection must persist before we publish (reduces noise)
CONFIRMATION_FRAMES = 5

# Template image paths (relative to this file).  Place cropped symbol images
# here; leave absent to use the colour-ratio fallback.
TEMPLATE_DIR = os.path.join(os.path.dirname(__file__), "gate_templates")
SURVEY_TEMPLATES  = ["compass.png", "hammer.png"]
RESCUE_TEMPLATES  = ["lifering.png", "sos.png"]


class GateDetector(Node):

    def __init__(self):
        super().__init__("gate_detector")

        self.bridge = CvBridge()

        # Camera intrinsics (filled in from CameraInfo)
        self.fx = self.fy = self.cx = self.cy = None
        self.camera_yaw_offset = 0.0   # radians; set if camera is not forward-facing

        # Latest robot pose from odometry (used to transform to world frame)
        self.robot_pose = None

        # Detection state
        self._confirm_count = 0
        self._last_gate_pose: PoseStamped | None = None
        self._last_side: str | None = None
        self._published = False          # only publish once per mission task

        # ORB templates
        self.orb = cv2.ORB_create(nfeatures=500)
        self.bf  = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.survey_templates = self._load_templates(SURVEY_TEMPLATES)
        self.rescue_templates = self._load_templates(RESCUE_TEMPLATES)

        # Subscribers
        self.create_subscription(Image,      "/camera/color/image_raw",        self._image_cb,    10)
        self.create_subscription(Image,      "/camera/aligned_depth_to_color/image_raw", self._depth_cb, 10)
        self.create_subscription(CameraInfo, "/camera/color/camera_info",      self._info_cb,     10)
        self.create_subscription(Odometry,   "/world/pose",                    self._odom_cb,     10)

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, "/perception/gate/pose", 10)
        self.side_pub = self.create_publisher(String,      "/perception/gate/side", 10)

        # Cached frames
        self._depth_frame: np.ndarray | None = None

        self.get_logger().info("GateDetector started.")

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------

    def _info_cb(self, msg: CameraInfo):
        if self.fx is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]

    def _depth_cb(self, msg: Image):
        self._depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def _odom_cb(self, msg: Odometry):
        self.robot_pose = msg

    def _image_cb(self, msg: Image):
        if self._published:
            return   # gate already handed off to mission executor

        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        result = self._detect_gate(bgr)
        if result is None:
            self._confirm_count = 0
            return

        gate_bbox, side = result
        self._confirm_count += 1

        if self._confirm_count < CONFIRMATION_FRAMES:
            return

        # Build and publish gate pose
        pose = self._bbox_to_pose(gate_bbox, bgr.shape)
        if pose is None:
            return

        self._last_gate_pose = pose
        self._last_side = side
        self._published = True

        self.pose_pub.publish(pose)

        side_msg = String()
        side_msg.data = side
        self.side_pub.publish(side_msg)

        self.get_logger().info(
            f"Gate detected! Side chosen: {side}  "
            f"pos=({pose.pose.position.x:.2f}, "
            f"{pose.pose.position.y:.2f}, "
            f"{pose.pose.position.z:.2f})"
        )

    # ------------------------------------------------------------------
    # Core detection
    # ------------------------------------------------------------------

    def _detect_gate(self, bgr: np.ndarray):
        """
        Returns (gate_bbox, side_str) or None.
        gate_bbox = (x, y, w, h) pixel bounding box of the full gate opening.
        side_str  = "survey_repair" | "search_rescue"
        """
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        red_mask   = self._red_mask(hsv)
        black_mask = cv2.inRange(hsv, BLACK_RANGE[0], BLACK_RANGE[1])

        # Combined panel mask
        panel_mask = cv2.bitwise_or(red_mask, black_mask)
        panel_mask = cv2.morphologyEx(panel_mask, cv2.MORPH_CLOSE,
                                      np.ones((7, 7), np.uint8))

        contours, _ = cv2.findContours(panel_mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        # Keep tall, reasonably narrow contours (gate leg panels)
        legs = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < MIN_PANEL_AREA:
                continue
            x, y, w, h = cv2.boundingRect(c)
            aspect = h / max(w, 1)
            if aspect < 1.2:          # must be taller than wide
                continue
            legs.append((x, y, w, h, area))

        if len(legs) < 2:
            return None

        # Pick the two largest legs by area
        legs.sort(key=lambda l: l[4], reverse=True)
        leg_left, leg_right = sorted(legs[:2], key=lambda l: l[0])

        # Gate bounding box spans from left leg to right leg
        gx = leg_left[0]
        gy = min(leg_left[1], leg_right[1])
        gw = (leg_right[0] + leg_right[2]) - gx
        gh = max(leg_left[1] + leg_left[3], leg_right[1] + leg_right[3]) - gy

        if gw <= 0 or gh <= 0:
            return None

        # Identify center divider: tall thin red band between the legs
        divider_x = self._find_divider_x(red_mask, leg_left, leg_right)

        # Classify left/right halves
        side = self._classify_side(bgr, gx, gy, gw, gh, divider_x)

        return (gx, gy, gw, gh), side

    def _red_mask(self, hsv: np.ndarray) -> np.ndarray:
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lo, hi in RED_RANGES:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo, hi))
        return mask

    def _find_divider_x(self, red_mask, leg_left, leg_right):
        """Return pixel x of the centre divider, or midpoint if not found."""
        mid_x = (leg_left[0] + leg_left[2] + leg_right[0]) // 2

        # Look for the tallest vertical red stripe between the two legs
        search_x1 = leg_left[0] + leg_left[2]
        search_x2 = leg_right[0]
        if search_x2 <= search_x1:
            return mid_x

        region = red_mask[:, search_x1:search_x2]
        col_sums = region.sum(axis=0)
        if col_sums.max() == 0:
            return mid_x

        return search_x1 + int(col_sums.argmax())

    def _classify_side(self, bgr, gx, gy, gw, gh, divider_x):
        """
        Determine which side carries Survey & Repair vs Search & Rescue.
        Returns "survey_repair" for the left half or "search_rescue", etc.,
        based on symbol matching (ORB) or colour heuristic fallback.
        """
        h, w = bgr.shape[:2]

        left_roi  = bgr[gy:gy+gh, gx:divider_x]
        right_roi = bgr[gy:gy+gh, divider_x:gx+gw]

        # --- Try ORB template matching first ---
        if self.survey_templates or self.rescue_templates:
            left_role  = self._match_templates(left_roi)
            right_role = self._match_templates(right_roi)

            if left_role == "survey_repair" or right_role == "search_rescue":
                return "survey_repair"   # pass through left
            if left_role == "search_rescue" or right_role == "survey_repair":
                return "search_rescue"   # pass through right

        # --- Colour heuristic fallback ---
        # Right leg: RED top / BLACK bottom → Survey side has red on top-right
        # Use the relative red pixel density in the upper half of each ROI.
        def upper_red_ratio(roi):
            if roi.size == 0:
                return 0.0
            half = roi[:roi.shape[0]//2]
            hsv  = cv2.cvtColor(half, cv2.COLOR_BGR2HSV)
            mask = self._red_mask(hsv)
            return mask.sum() / max(half.size, 1)

        left_red  = upper_red_ratio(left_roi)
        right_red = upper_red_ratio(right_roi)

        # Right leg is RED on top → "Red Right Above" is the Survey side per
        # competition description.  Pass through the side with the preferred
        # symbol; default: if right upper is redder → survey is on right.
        if right_red >= left_red:
            # Survey & Repair is on the right side
            # mission_executor will pick side; we just report which is which
            return "survey_on_right"
        else:
            return "survey_on_left"

    def _match_templates(self, roi):
        """Return role string if a template matches, else None."""
        if roi.size == 0:
            return None
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        kp2, des2 = self.orb.detectAndCompute(gray, None)
        if des2 is None:
            return None

        best_role   = None
        best_count  = 8   # minimum good matches to accept

        for role, templates in [("survey_repair", self.survey_templates),
                                 ("search_rescue", self.rescue_templates)]:
            for _, des1 in templates:
                if des1 is None:
                    continue
                matches = self.bf.match(des1, des2)
                good    = [m for m in matches if m.distance < 60]
                if len(good) > best_count:
                    best_count = len(good)
                    best_role  = role

        return best_role

    def _load_templates(self, filenames):
        """Load ORB descriptors for template images. Returns list of (img, des)."""
        results = []
        for fn in filenames:
            path = os.path.join(TEMPLATE_DIR, fn)
            if not os.path.exists(path):
                continue
            img  = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
            if img is None:
                continue
            kp, des = self.orb.detectAndCompute(img, None)
            results.append((img, des))
            self.get_logger().info(f"Loaded template: {fn}")
        return results

    # ------------------------------------------------------------------
    # 3-D pose estimation
    # ------------------------------------------------------------------

    def _bbox_to_pose(self, bbox, img_shape) -> PoseStamped | None:
        """
        Deproject the gate centre pixel through the depth frame to get a
        metric 3-D position, then transform into the world frame using the
        latest odometry.
        """
        if self.fx is None:
            self.get_logger().warn("No camera intrinsics yet — cannot compute pose.")
            return None

        gx, gy, gw, gh = bbox
        cx_px = gx + gw // 2
        cy_px = gy + gh // 2

        # --- Depth lookup ---
        depth_m = self._sample_depth(cx_px, cy_px)
        if depth_m is None or depth_m <= 0.1:
            self.get_logger().warn("Invalid depth reading — gate too close or depth not ready.")
            return None

        # --- Deproject pixel → camera-frame 3-D point ---
        # Camera frame: +Z forward, +X right, +Y down
        x_cam = (cx_px - self.cx) * depth_m / self.fx
        y_cam = (cy_px - self.cy) * depth_m / self.fy
        z_cam = depth_m

        # Rotate to ROS body frame (+X forward, +Y left, +Z up)
        x_body =  z_cam
        y_body = -x_cam
        z_body = -y_cam

        # --- Transform to world frame using robot odometry ---
        if self.robot_pose is not None:
            rp = self.robot_pose.pose.pose
            rot = Rotation.from_quat([
                rp.orientation.x, rp.orientation.y,
                rp.orientation.z, rp.orientation.w,
            ])
            world_pos = rot.apply([x_body, y_body, z_body]) + np.array([
                rp.position.x, rp.position.y, rp.position.z
            ])
            # Gate orientation: robot yaw only (gate is vertical)
            robot_yaw = rot.as_euler("xyz", degrees=False)[2]
        else:
            world_pos = np.array([x_body, y_body, z_body])
            robot_yaw = 0.0

        # Gate forward direction = robot yaw (we're facing the gate)
        gate_quat = Rotation.from_euler("z", robot_yaw).as_quat()

        pose = PoseStamped()
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = float(world_pos[0])
        pose.pose.position.y = float(world_pos[1])
        pose.pose.position.z = float(world_pos[2])
        pose.pose.orientation.x = gate_quat[0]
        pose.pose.orientation.y = gate_quat[1]
        pose.pose.orientation.z = gate_quat[2]
        pose.pose.orientation.w = gate_quat[3]
        return pose

    def _sample_depth(self, cx, cy, radius=10):
        """Return median depth (metres) in a small patch, or None."""
        if self._depth_frame is None:
            return None
        h, w = self._depth_frame.shape[:2]
        x1, x2 = max(cx - radius, 0), min(cx + radius, w)
        y1, y2 = max(cy - radius, 0), min(cy + radius, h)
        patch = self._depth_frame[y1:y2, x1:x2].astype(np.float32)
        # RealSense depth is in millimetres
        patch = patch[patch > 0]
        if patch.size == 0:
            return None
        return float(np.median(patch)) / 1000.0   # convert mm → m


def main(args=None):
    rclpy.init(args=args)
    node = GateDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
