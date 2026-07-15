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
    """Median-of-window + EMA smoothing behind a lock-on gate.

    SEARCHING: detections accumulate in a sliding time window; the filter
    exposes nothing until `confirm_hits` of them agree within a range-scaled
    radius. LOCKED: agreeing detections update the smoothed goal; outliers
    are ignored and do not refresh staleness, so a stream of false positives
    can neither drag a lock nor keep a dead one alive.
    """

    def __init__(self, window=5, alpha=0.3, max_jump_m=3.0, stale_sec=2.0,
                 confirm_hits=10, confirm_window_sec=1.0,
                 lock_radius_min_m=0.05, lock_radius_frac=0.03):
        self.window = deque(maxlen=window)
        self.alpha = alpha
        self.max_jump_m = max_jump_m
        self.stale_sec = stale_sec
        self.confirm_hits = confirm_hits
        self.confirm_window_sec = confirm_window_sec
        self.lock_radius_min_m = lock_radius_min_m
        self.lock_radius_frac = lock_radius_frac
        self.hits = deque()  # (t, p) candidates while SEARCHING
        self.locked = False
        self.ema = None
        self.last_t = None

    def _agree_radius(self, point, sub_pos):
        if sub_pos is None:
            return self.lock_radius_min_m
        rng = float(np.linalg.norm(np.asarray(point) - np.asarray(sub_pos)))
        return max(self.lock_radius_min_m, self.lock_radius_frac * rng)

    def _unlock(self):
        self.locked = False
        self.hits.clear()
        self.window.clear()
        self.ema = None
        self.last_t = None

    def update(self, p_odom, t, sub_pos=None):
        p = np.asarray(p_odom, dtype=float)
        if not np.all(np.isfinite(p)):
            return None

        if self.locked and self.last_t is not None and t - self.last_t > self.stale_sec:
            self._unlock()

        if not self.locked:
            self.hits.append((t, p))
            while self.hits and t - self.hits[0][0] > self.confirm_window_sec:
                self.hits.popleft()
            if len(self.hits) < self.confirm_hits:
                return None
            pts = np.stack([h[1] for h in self.hits])
            med = np.median(pts, axis=0)
            radius = self._agree_radius(med, sub_pos)
            if np.max(np.linalg.norm(pts - med, axis=1)) > radius:
                return None
            self.locked = True
            self.hits.clear()
            self.window.clear()
            self.window.append(med)
            self.ema = med
            self.last_t = t
            return self.ema.copy()

        # LOCKED: reject outliers without refreshing staleness
        radius = self._agree_radius(self.ema, sub_pos)
        d = float(np.linalg.norm(p - self.ema))
        if d > radius or d > self.max_jump_m:
            return None
        self.window.append(p)
        med = np.median(np.stack(self.window), axis=0)
        self.ema = self.alpha * med + (1.0 - self.alpha) * self.ema
        self.last_t = t
        return self.ema.copy()

    def get(self, t):
        if not self.locked or self.ema is None or self.last_t is None:
            return None
        if t - self.last_t > self.stale_sec:
            self._unlock()
            return None
        return self.ema.copy()
