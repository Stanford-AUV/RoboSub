"""Astigmatic division-model distortion for the OAK-D behind the tube.

Normalized coords: x = (u - w/2)/s, y = (v - h/2)/s with
s = 0.5*hypot(w, h), so params are resolution-independent.
Model: r2 = x^2 + (sy*y)^2; D = 1 + k1*r2 + k2*r2^2;
undistorted = distorted / D.
"""
import math
import os
import sys

import numpy as np
import yaml


def _scale(w, h):
    return 0.5 * math.hypot(w, h)


def to_norm(pts_px, w, h):
    pts_px = np.asarray(pts_px, dtype=np.float64)
    s = _scale(w, h)
    return (pts_px - np.array([w / 2.0, h / 2.0])) / s


def to_px(pts_norm, w, h):
    pts_norm = np.asarray(pts_norm, dtype=np.float64)
    s = _scale(w, h)
    return pts_norm * s + np.array([w / 2.0, h / 2.0])


def _D(pts, k1, k2, sy):
    r2 = pts[:, 0] ** 2 + (sy * pts[:, 1]) ** 2
    return 1.0 + k1 * r2 + k2 * r2 * r2


def undistort_pts(pts_norm, k1, k2, sy):
    pts_norm = np.asarray(pts_norm, dtype=np.float64)
    return pts_norm / _D(pts_norm, k1, k2, sy)[:, None]


def distort_pts(pts_norm, k1, k2, sy, iters=25):
    pts_norm = np.asarray(pts_norm, dtype=np.float64)
    pd = pts_norm.copy()
    for _ in range(iters):
        pd = pts_norm * _D(pd, k1, k2, sy)[:, None]
    return pd


def build_remap(w, h, params):
    """Maps for cv2.remap: output = undistorted image of the input."""
    ys, xs = np.mgrid[0:h, 0:w].astype(np.float64)
    grid = np.stack([xs.ravel(), ys.ravel()], axis=1)
    src = to_px(
        distort_pts(
            to_norm(grid, w, h), params["k1"], params["k2"], params["sy"]
        ),
        w,
        h,
    )
    bad = ~np.isfinite(src).all(axis=1)
    if bad.any():
        print(
            f"build_remap: {int(bad.sum())} non-finite remap points — "
            "falling back to identity mapping for those pixels",
            file=sys.stderr,
        )
        src[bad] = grid[bad]
    return (
        src[:, 0].reshape(h, w).astype(np.float32),
        src[:, 1].reshape(h, w).astype(np.float32),
    )


def save_params(path, params):
    with open(path, "w") as f:
        yaml.safe_dump({k: params[k] for k in ("k1", "k2", "sy", "width", "height")}, f)


def load_params(path):
    if not os.path.exists(path):
        return None
    try:
        with open(path) as f:
            p = yaml.safe_load(f)
        return {
            "k1": float(p["k1"]),
            "k2": float(p["k2"]),
            "sy": float(p["sy"]),
            "width": int(p["width"]),
            "height": int(p["height"]),
        }
    except Exception as e:
        print(f"load_params: failed to parse {path}: {e}", file=sys.stderr)
        return None
