import math
import sys

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge


# --- bottom-line orientation detection --------------------------------------
# Dead simple, per the field-tested recipe: find lines PERMISSIVELY (low
# Canny/Hough thresholds — the bar is LOW). Then two filters:
#   1) SPATIAL: keep the densest neighborhood of segments (the mat is one
#      compact region; scattered caustic/tile edges elsewhere drop out),
#   2) ANGLE: within that group, keep the biggest cluster of near-identical
#      angles (length-weighted), throw everything else out, average.
PROC_WIDTH = 960          # process at this width (scale-invariant params)
MIN_SEG_LEN = 25          # px at PROC_WIDTH
MAX_LINE_GAP = 10
HOUGH_THRESHOLD = 25
SPATIAL_RADIUS_FRAC = 0.3  # neighborhood radius, fraction of frame short side
CLUSTER_TOL = math.radians(5)  # segments within this of each other agree
MIN_CLUSTER_LINES = 8     # publish only with at least this many agreeing
MIN_CLUSTER_LEN = 300.0   # ...and this much total length (px at PROC_WIDTH)


def estimate_line_angle(img, debug_out=None):
    """Angle of the dominant line direction in the camera frame.

    Returns (angle, n_lines) or (None, 0), where n_lines is the number
    of segments in the winning angle cluster.

    Angle convention: visual (y-up) angle in [-90, 90) deg (a line is
    180-deg ambiguous). Positive angle = positive slope as a human would
    plot it. The camera is aligned with the vehicle heading, so a
    positive slope means the vehicle is rotated CLOCKWISE (viewed from
    above) relative to the line.
    """
    scale = PROC_WIDTH / img.shape[1]
    small = cv2.resize(img, (PROC_WIDTH, int(img.shape[0] * scale)))
    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    med = float(np.median(blur))
    if med < 1:
        return None, 0
    edges = cv2.Canny(blur, 0.3 * med, 0.7 * med)
    segs = cv2.HoughLinesP(
        edges,
        rho=1,
        theta=np.pi / 180,
        threshold=HOUGH_THRESHOLD,
        minLineLength=MIN_SEG_LEN,
        maxLineGap=MAX_LINE_GAP,
    )
    if segs is None:
        return None, 0

    segs = segs[:, 0].astype(float)
    length = np.hypot(segs[:, 2] - segs[:, 0], segs[:, 3] - segs[:, 1])
    # image y grows downward; negate dy for visual angles, fold mod 180
    theta = np.arctan2(-(segs[:, 3] - segs[:, 1]), segs[:, 2] - segs[:, 0])
    theta = theta % np.pi
    mids = np.stack(
        [(segs[:, 0] + segs[:, 2]) / 2.0, (segs[:, 1] + segs[:, 3]) / 2.0],
        axis=1,
    )

    # 1) Spatial cluster: the segment whose neighborhood holds the most
    # total segment LENGTH seeds the group; only its neighbors advance.
    radius = SPATIAL_RADIUS_FRAC * min(small.shape[:2])
    dist = np.linalg.norm(mids[:, None, :] - mids[None, :, :], axis=2)
    near = dist < radius
    seed = int(np.argmax((length[None, :] * near).sum(axis=1)))
    spatial = near[seed]

    # 2) Biggest angle cluster within the spatial group: each segment's
    # support is the total LENGTH of group segments within CLUSTER_TOL of
    # it (circular, mod 180); best wins.
    d = np.abs(theta[:, None] - theta[None, :])
    d = np.minimum(d, np.pi - d)
    support = (length[None, :] * (d < CLUSTER_TOL) * spatial[None, :]).sum(
        axis=1
    )
    support[~spatial] = -1.0
    best = int(np.argmax(support))
    inliers = spatial & (d[best] < CLUSTER_TOL)
    n_lines = int(inliers.sum())
    total_len = float(length[inliers].sum())
    if n_lines < MIN_CLUSTER_LINES or total_len < MIN_CLUSTER_LEN:
        return None, 0

    # Length-weighted circular mean of the cluster (double angles: mod 180).
    a2 = 2.0 * theta[inliers]
    w = length[inliers]
    angle = math.atan2(
        float((w * np.sin(a2)).sum()), float((w * np.cos(a2)).sum())
    ) / 2.0
    angle = (angle + np.pi / 2) % np.pi - np.pi / 2  # [-90, 90)

    if debug_out is not None:
        s = 1.0 / scale
        # green = winning cluster (used for the angle); yellow = in the
        # spatial group but wrong angle; red = outside the spatial group
        for (x1, y1, x2, y2), ok, sp in zip(
            segs.astype(int), inliers, spatial
        ):
            color = (0, 255, 0) if ok else ((0, 255, 255) if sp else (0, 0, 255))
            cv2.line(
                debug_out,
                (int(x1 * s), int(y1 * s)),
                (int(x2 * s), int(y2 * s)),
                color,
                2,
            )
        c = mids[inliers].mean(axis=0) * s
        dvec = 0.45 * min(debug_out.shape[:2]) * np.array(
            [math.cos(angle), -math.sin(angle)]
        )
        cv2.line(
            debug_out,
            tuple((c - dvec).astype(int)),
            tuple((c + dvec).astype(int)),
            (255, 0, 255),
            4,
        )
        cv2.putText(
            debug_out,
            f"{math.degrees(angle):+.1f} deg "
            f"({n_lines}/{len(segs)} lines, {total_len:.0f}px)",
            (10, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.2,
            (255, 0, 255),
            2,
        )

    return angle, n_lines


class HeadingCorrector(Node):
    def __init__(self, camera_key):
        super().__init__(f"{camera_key}_heading_corrector")

        self.camera_key = camera_key
        self.bridge = CvBridge()

        rgb_topic = f"/camera/{self.camera_key}/rgb"
        self.camera_rgb = self.create_subscription(
            Image, rgb_topic, self.image_callback, 10
        )

        # Current believed yaw (IMU): used to resolve the 2-fold ambiguity
        # of a line observation (facing along vs against it).
        self._imu_yaw = None
        self.create_subscription(
            PoseWithCovarianceStamped, "/rotation", self.rotation_callback, 10
        )

        # Separate topic from /rotation (imu.py owns that): absolute yaw,
        # snapped to the line-axis candidate nearest the believed yaw.
        self.correction_publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/heading_correction", 10
        )

        self.get_logger().info(f"Subscribed to {rgb_topic}")

    def rotation_callback(self, msg):
        q = msg.pose.pose.orientation
        self._imu_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def image_callback(self, msg):
        if self._imu_yaw is None:
            return  # can't resolve the 2-fold ambiguity without a yaw yet
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            self.get_logger().error(
                f"Error decoding rgb image from {self.camera_key}"
            )
            return

        angle, support = estimate_line_angle(img)
        if angle is None:
            return

        # Positive visual slope = vehicle rotated clockwise = NEGATIVE yaw
        # in FLU, so yaw relative to the line is -angle. A line can't
        # distinguish heading 0/180 along it, so snap to whichever of the
        # 2 candidates is closest to the current believed yaw.
        yaw_rel = -angle
        candidates = [
            (yaw_rel + k * math.pi + math.pi) % (2.0 * math.pi) - math.pi
            for k in range(2)
        ]
        yaw_abs = min(
            candidates,
            key=lambda c: abs(
                (c - self._imu_yaw + math.pi) % (2.0 * math.pi) - math.pi
            ),
        )

        out = PoseWithCovarianceStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = "odom"
        out.pose.pose.orientation.z = math.sin(yaw_abs / 2.0)
        out.pose.pose.orientation.w = math.cos(yaw_abs / 2.0)
        # Bigger agreeing cluster -> more confident.
        yaw_var = 0.1 / min(support, 40)
        cov = np.zeros(36)
        cov[35] = yaw_var
        out.pose.covariance = cov.tolist()
        self.correction_publisher.publish(out)


def main(args=None):
    # Offline test mode: pass image paths to run the detector on photos.
    #   python3 heading_corrector.py pool1.jpg pool2.jpg
    # Writes <name>_debug.jpg next to each input and prints the angle.
    image_paths = [a for a in sys.argv[1:] if not a.startswith("-")]
    if image_paths:
        for path in image_paths:
            img = cv2.imread(path)
            if img is None:
                print(f"{path}: could not read")
                continue
            debug = img.copy()
            angle, support = estimate_line_angle(img, debug_out=debug)
            out_path = path.rsplit(".", 1)[0] + "_debug.jpg"
            cv2.imwrite(out_path, debug)
            if angle is None:
                print(f"{path}: no dominant line direction -> {out_path}")
            else:
                print(
                    f"{path}: {math.degrees(angle):+.1f} deg "
                    f"(vehicle rotated {'CW' if angle > 0 else 'CCW'} "
                    f"{abs(math.degrees(angle)):.1f} deg, "
                    f"{support} lines agree) -> {out_path}"
                )
        return

    rclpy.init(args=args)
    node = HeadingCorrector("oak_0")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
