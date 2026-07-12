"""Fit tube undistortion from pool tile-grid photos (plumb-line method).

Fit:    python3 fit_undistortion.py img1.png img2.jpg
Apply:  python3 fit_undistortion.py --apply photo.jpg [--debug]

Frames are filtered by their CENTER CROP: if the central 50%x50% has no
dominant straight-line cluster, the frame is skipped. The loss is line
straightness only; center-vs-full angle agreement is reported as a
validation metric (perspective changes angles but not straightness).
"""
import argparse
import math
import os
import sys

import cv2
import numpy as np
from scipy.optimize import least_squares
from tqdm import tqdm

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "..", "src", "perception"))
sys.path.insert(
    0, os.path.join(_HERE, "..", "..", "src", "perception", "perception", "nodes")
)
from perception.utils.undistort import (  # noqa: E402
    build_remap,
    load_params,
    save_params,
    to_norm,
    undistort_pts,
)
from heading_corrector import PROC_WIDTH, estimate_line_angle  # noqa: E402

DEFAULT_YAML = os.path.join(_HERE, "..", "..", "config", "undistort_oak0.yaml")
CHAIN_LINK_DIST = 15.0     # px between endpoints to link segments
CHAIN_LINK_ANGLE = math.radians(8)
MIN_CHAIN_SEGS = 3
MIN_CHAIN_SPAN = 250.0     # px end-to-end
MIN_CENTER_LINES = 20      # center-crop cluster size to accept a frame


def detect_segments(img_bgr):
    """Permissive Hough at PROC_WIDTH scale — same recipe as the corrector."""
    scale = PROC_WIDTH / img_bgr.shape[1]
    small = cv2.resize(img_bgr, (PROC_WIDTH, int(img_bgr.shape[0] * scale)))
    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
    blur = cv2.fastNlMeansDenoising(gray, None, 7, 7, 21)
    med = float(np.median(blur))
    if med < 1:
        return np.zeros((0, 4)), small.shape
    edges = cv2.Canny(blur, 0.15 * med, 0.4 * med)
    segs = cv2.HoughLinesP(
        edges, rho=1, theta=np.pi / 180, threshold=45,
        minLineLength=40, maxLineGap=8,
    )
    if segs is None:
        return np.zeros((0, 4)), small.shape
    return segs[:, 0].astype(float), small.shape


def center_crop_score(img_bgr):
    h, w = img_bgr.shape[:2]
    crop = img_bgr[h // 4: 3 * h // 4, w // 4: 3 * w // 4]
    return estimate_line_angle(crop)


def _seg_angle(s):
    return math.atan2(s[3] - s[1], s[2] - s[0]) % math.pi


def build_chains(segs):
    n = len(segs)
    if n == 0:
        return []
    parent = list(range(n))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    ends = segs.reshape(n, 2, 2)
    angles = np.array([_seg_angle(s) for s in segs])
    for i in range(n):
        for j in range(i + 1, n):
            da = abs(angles[i] - angles[j])
            da = min(da, math.pi - da)
            if da > CHAIN_LINK_ANGLE:
                continue
            d = np.linalg.norm(
                ends[i][:, None, :] - ends[j][None, :, :], axis=2
            ).min()
            if d < CHAIN_LINK_DIST:
                parent[find(i)] = find(j)

    groups = {}
    for i in range(n):
        groups.setdefault(find(i), []).append(i)
    chains = []
    for idx in groups.values():
        if len(idx) < MIN_CHAIN_SEGS:
            continue
        pts = ends[idx].reshape(-1, 2)
        span = np.linalg.norm(pts.max(axis=0) - pts.min(axis=0))
        if span < MIN_CHAIN_SPAN:
            continue
        chains.append(pts)
    return chains


def _chain_residuals(pts_norm_und, scale_px):
    """Perpendicular distances (px) of undistorted points to their TLS line."""
    c = pts_norm_und - pts_norm_und.mean(axis=0)
    _, _, vt = np.linalg.svd(c, full_matrices=False)
    return (c @ vt[1]) * scale_px


def fit_params(chains_by_image, w, h):
    scale_px = 0.5 * math.hypot(w, h)
    norm_chains = [
        to_norm(pts, w, h)
        for chains in chains_by_image.values()
        for pts in chains
    ]

    def residuals(x):
        k1, k2, sy = x
        out = []
        for pts in norm_chains:
            und = undistort_pts(pts, k1, k2, sy)
            out.append(_chain_residuals(und, scale_px))
        return np.concatenate(out)

    res = least_squares(
        residuals,
        x0=[0.0, 0.0, 1.0],
        bounds=([-0.5, -0.5, 0.8], [0.5, 0.5, 1.25]),
        loss="soft_l1",
        f_scale=2.0,
    )
    k1, k2, sy = res.x
    return {"k1": float(k1), "k2": float(k2), "sy": float(sy),
            "width": int(w), "height": int(h)}


def straightness_rms(chains, params, w, h):
    scale_px = 0.5 * math.hypot(w, h)
    r = np.concatenate([
        _chain_residuals(
            undistort_pts(to_norm(pts, w, h), params["k1"], params["k2"], params["sy"]),
            scale_px,
        )
        for pts in chains
    ])
    return float(np.sqrt((r ** 2).mean()))
