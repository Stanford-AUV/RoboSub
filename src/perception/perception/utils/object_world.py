"""Camera-frame detection -> odom-frame goal: transforms + smoothing filter."""

from collections import deque

import numpy as np
from scipy.spatial.transform import Rotation

# Optical frame (x right, y down, z forward) -> ROS body (x fwd, y left, z up).
OPTICAL_TO_ROS = np.array(
    [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]
)


def camera_to_base(p_cam, cam_translation, cam_rpy_deg):
    """Optical-frame point -> base_link. cam_rpy_deg is the mount rotation of
    the camera body (ROS convention) in base_link; identity = forward-looking."""
    p_ros = OPTICAL_TO_ROS @ np.asarray(p_cam, dtype=float)
    rot = Rotation.from_euler("xyz", cam_rpy_deg, degrees=True).as_matrix()
    return rot @ p_ros + np.asarray(cam_translation, dtype=float)


def base_to_odom(p_base, base_pos, base_quat_xyzw):
    """base_link point -> odom, given the EKF pose of base_link in odom."""
    rot = Rotation.from_quat(base_quat_xyzw).as_matrix()
    return rot @ np.asarray(p_base, dtype=float) + np.asarray(base_pos, dtype=float)


class DetectionFilter:
    """Median-of-window then EMA smoothing with outlier gate and staleness."""

    def __init__(self, window=5, alpha=0.3, max_jump_m=3.0, stale_sec=2.0):
        self.window = deque(maxlen=window)
        self.alpha = alpha
        self.max_jump_m = max_jump_m
        self.stale_sec = stale_sec
        self.ema = None
        self.last_t = None

    def update(self, p_odom, t):
        p = np.asarray(p_odom, dtype=float)
        if not np.all(np.isfinite(p)):
            return None
        if self.ema is not None and np.linalg.norm(p - self.ema) > self.max_jump_m:
            return None
        self.window.append(p)
        med = np.median(np.stack(self.window), axis=0)
        if self.ema is None:
            self.ema = med
        else:
            self.ema = self.alpha * med + (1.0 - self.alpha) * self.ema
        self.last_t = t
        return self.ema.copy()

    def get(self, t):
        if self.ema is None or self.last_t is None:
            return None
        if t - self.last_t > self.stale_sec:
            return None
        return self.ema.copy()
