import math
import os
import sys

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge

try:
    from perception.utils.undistort import build_remap, load_params
except ImportError:  # run as a plain script (offline photo mode)
    sys.path.insert(
        0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
    )
    from perception.utils.undistort import build_remap, load_params

UNDISTORT_YAML = os.path.expanduser("~/RoboSub/config/undistort_oak0.yaml")
_undistort_params = load_params(UNDISTORT_YAML)
_undistort_maps = {}


def _get_undistort_maps(w, h):
    if _undistort_params is None:
        return None
    if (w, h) not in _undistort_maps:
        _undistort_maps[(w, h)] = build_remap(w, h, _undistort_params)
    return _undistort_maps[(w, h)]


# --- bottom-line orientation detection --------------------------------------
# Dead simple, per the field-tested recipe: find lines PERMISSIVELY (low
# Canny/Hough thresholds — the bar is LOW). Then two filters:
#   SPATIAL + ANGLE jointly: keep the biggest (length-weighted) group of
#   segments that are close together AND share nearly the same angle,
#   throw everything else out, average. Close-together lines from the same
#   physical feature (mat border, lane line, grout) satisfy both; caustic
#   speckle and icon edges never do.
PROC_WIDTH = 960          # process at this width (scale-invariant params)
MIN_SEG_LEN = 55          # px at PROC_WIDTH
MAX_LINE_GAP = 10
HOUGH_THRESHOLD = 45
SPATIAL_RADIUS_FRAC = 0.3  # neighborhood radius, fraction of frame short side
CLUSTER_TOL = math.radians(1)  # segments within this of each other agree
MIN_CLUSTER_LINES = 20    # publish only with at least this many agreeing
                          # (real tile/mat frames give 60+; pure caustic
                          # false clusters topped out at ~27 in testing)
MIN_CLUSTER_LEN = 400.0   # ...and this much total length (px at PROC_WIDTH)
MAX_EKF_DISAGREE = math.radians(1)  # drop readings further than this from
                                     # the current believed yaw: caustic
                                     # clusters point anywhere, real lines
                                     # roughly agree with the EKF already
PROCESS_HZ = 3.0          # denoise costs ~170 ms/frame on the Orin; skip
                          # camera frames beyond this rate


def estimate_line_angle(img, debug_out=None, undistort=True):
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
    if undistort:
        maps = _get_undistort_maps(small.shape[1], small.shape[0])
        if maps is not None:
            small = cv2.remap(
                small, maps[0], maps[1], cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_REPLICATE,
            )
    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
    # Non-local means denoise (~170 ms/frame on the Orin — the node
    # throttles to PROCESS_HZ to compensate): kills caustic speckle
    # before it reaches Canny, roughly halving spurious segments, while
    # keeping real edges crisp. Contrast boosting (CLAHE/unsharp) was
    # tested and REJECTED: it amplifies caustics into hundreds of
    # coherent false lines on line-free frames.
    blur = cv2.fastNlMeansDenoising(gray, None, 7, 7, 21)
    med = float(np.median(blur))
    if med < 1:
        return None, 0
    # Low relative-to-median thresholds on purpose: the blue tile grout is
    # LOW contrast (vs the high-contrast printed icons), and it only wins
    # the cluster vote if its faint edges make it into the pool at all.
    edges = cv2.Canny(blur, 0.15 * med, 0.4 * med)
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

    # Joint cluster: a segment's supporters are the segments that are BOTH
    # near it (spatially) AND at nearly the same angle. The segment with
    # the most supporting LENGTH seeds the group; its supporters are the
    # inliers. Close-together lines from the same physical feature agree
    # in both; caustics agree in neither.
    radius = SPATIAL_RADIUS_FRAC * min(small.shape[:2])
    near = (
        np.linalg.norm(mids[:, None, :] - mids[None, :, :], axis=2) < radius
    )
    d = np.abs(theta[:, None] - theta[None, :])
    d = np.minimum(d, np.pi - d)
    agree = near & (d < CLUSTER_TOL)
    support = (length[None, :] * agree).sum(axis=1)
    best = int(np.argmax(support))
    inliers = agree[best]
    spatial = near[best]  # for the debug overlay
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

        if _undistort_params is None:
            self.get_logger().warn(
                f"no undistortion calibration at {UNDISTORT_YAML} — "
                "running on distorted frames"
            )
        else:
            self.get_logger().info("tube undistortion active")

        rgb_topic = f"/camera/{self.camera_key}/rgb"
        self.camera_rgb = self.create_subscription(
            Image, rgb_topic, self.image_callback, 10
        )

        # Current believed yaw (IMU): used to resolve the 2-fold ambiguity
        # of a line observation (facing along vs against it).
        self._imu_yaw = None
        self._last_process = 0.0
        # imu.py publishes the Xsens filtered orientation as sensor_msgs/Imu on
        # /imu/orientation (was PoseWithCovarianceStamped on /rotation).
        self.create_subscription(
            Imu, "/imu/orientation", self.rotation_callback, 10
        )

        # Separate topic from /imu/orientation (imu.py owns that): absolute yaw,
        # snapped to the line-axis candidate nearest the believed yaw.
        self.correction_publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/heading_correction", 10
        )

        self.get_logger().info(f"Subscribed to {rgb_topic}")

    def rotation_callback(self, msg):
        q = msg.orientation
        self._imu_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def image_callback(self, msg):
        if self._imu_yaw is None:
            return  # can't resolve the 2-fold ambiguity without a yaw yet
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_process < 1.0 / PROCESS_HZ:
            return
        self._last_process = now
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

        # Sanity gate: if even the best candidate disagrees with the
        # believed yaw by more than MAX_EKF_DISAGREE, this is a false lock
        # (caustics, occluder) — throw it out rather than fight the EKF.
        disagree = abs(
            (yaw_abs - self._imu_yaw + math.pi) % (2.0 * math.pi) - math.pi
        )
        if disagree > MAX_EKF_DISAGREE:
            self.get_logger().warn(
                f"heading correction {math.degrees(yaw_abs):+.1f} deg is "
                f"{math.degrees(disagree):.0f} deg from believed yaw — dropped"
            )
            return

        out = PoseWithCovarianceStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = "odom"
        out.pose.pose.orientation.z = math.sin(yaw_abs / 2.0)
        out.pose.pose.orientation.w = math.cos(yaw_abs / 2.0)
        # Bigger agreeing cluster -> more confident, but never overconfident:
        # even a clean detection is reasonably a few degrees off (blur,
        # caustics, mat not perfectly straight), so the floor is ~6 deg
        # std (support=40) rising to ~10 deg std at minimal support.
        yaw_var = 0.008 + 0.16 / min(support, 40)
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
