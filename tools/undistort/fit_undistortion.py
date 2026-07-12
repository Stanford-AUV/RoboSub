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
COLLINEAR_TOL = 5.0        # px perpendicular tolerance to link collinear segments


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


def _perp_dists(seg, pts):
    p0 = np.array(seg[:2])
    d = np.array(seg[2:]) - p0
    n = np.array([-d[1], d[0]]) / max(np.linalg.norm(d), 1e-9)
    return np.abs((pts - p0) @ n)


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
            if d >= CHAIN_LINK_DIST:
                continue
            # mutual collinearity gate: reject adjacent parallel lines
            if (_perp_dists(segs[i], ends[j]).max() > COLLINEAR_TOL
                    or _perp_dists(segs[j], ends[i]).max() > COLLINEAR_TOL):
                continue
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
            perp = _chain_residuals(und, scale_px)
            # per-chain normalization: a uniform image shrink no longer
            # reduces the loss
            c = und - und.mean(axis=0)
            _, _, vt = np.linalg.svd(c, full_matrices=False)
            proj = (c @ vt[0]) * scale_px
            L = proj.max() - proj.min()
            out.append(perp / max(L, 1.0) * 500.0)
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


def apply_to_image(path, params, debug=False):
    img = cv2.imread(path)
    if img is None:
        print(f"{path}: could not read")
        return
    h, w = img.shape[:2]
    map_x, map_y = build_remap(w, h, params)
    und = cv2.remap(img, map_x, map_y, cv2.INTER_LINEAR,
                    borderMode=cv2.BORDER_REPLICATE)
    out = path.rsplit(".", 1)[0] + "_undistorted.jpg"
    cv2.imwrite(out, und)
    print(f"{path} -> {out}")
    if debug:
        dbg = und.copy()
        angle, n = estimate_line_angle(und, debug_out=dbg)
        dbg_path = path.rsplit(".", 1)[0] + "_undistorted_debug.jpg"
        cv2.imwrite(dbg_path, dbg)
        print(f"  undistorted heading: "
              f"{'none' if angle is None else f'{math.degrees(angle):+.1f} deg'} "
              f"({n} lines) -> {dbg_path}")


def center_vs_full(img, params=None):
    """Angle disagreement (deg) between center crop and full frame."""
    if params is not None:
        h, w = img.shape[:2]
        map_x, map_y = build_remap(w, h, params)
        img = cv2.remap(img, map_x, map_y, cv2.INTER_LINEAR,
                        borderMode=cv2.BORDER_REPLICATE)
    a_c, _ = center_crop_score(img)
    a_f, _ = estimate_line_angle(img)
    if a_c is None or a_f is None:
        return None
    d = abs(a_c - a_f)
    return math.degrees(min(d, math.pi - d))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("images", nargs="+")
    ap.add_argument("--apply", action="store_true",
                    help="undistort images with the saved YAML instead of fitting")
    ap.add_argument("--debug", action="store_true")
    ap.add_argument("--yaml", default=DEFAULT_YAML)
    args = ap.parse_args()

    if args.apply:
        params = load_params(args.yaml)
        if params is None:
            sys.exit(f"no calibration at {args.yaml} — run fit mode first")
        for path in tqdm(args.images, desc="apply"):
            apply_to_image(path, params, debug=args.debug)
        return

    chains_by_image, shapes, kept_imgs = {}, set(), {}
    for path in tqdm(args.images, desc="filter+detect"):
        img = cv2.imread(path)
        if img is None:
            print(f"{path}: could not read — skipped")
            continue
        angle_c, n_c = center_crop_score(img)
        if angle_c is None or n_c < MIN_CENTER_LINES:
            print(f"{path}: center crop has no clean grid ({n_c} lines) — skipped")
            continue
        segs, shape = detect_segments(img)
        chains = build_chains(segs)
        if not chains:
            print(f"{path}: no usable line chains — skipped")
            continue
        print(f"{path}: accepted ({len(chains)} chains, "
              f"center angle {math.degrees(angle_c):+.1f} deg)")
        chains_by_image[path] = chains
        kept_imgs[path] = img
        shapes.add(shape[:2])

    if not chains_by_image:
        sys.exit("no frames passed the center-crop filter — nothing to fit")
    h, w = shapes.pop()  # all frames resized to PROC_WIDTH
    all_chains = [c for cs in chains_by_image.values() for c in cs]

    # Held-out check: fit on even chains, evaluate odd
    fit_cv = fit_params({"train": all_chains[0::2]}, w, h)
    print(f"held-out straightness RMS: "
          f"{straightness_rms(all_chains[1::2], fit_cv, w, h):.2f} px")

    params = fit_params(chains_by_image, w, h)
    ident = {"k1": 0.0, "k2": 0.0, "sy": 1.0, "width": w, "height": h}
    print(f"fitted: k1={params['k1']:+.4f} k2={params['k2']:+.4f} "
          f"sy={params['sy']:.4f}")
    print(f"straightness RMS: {straightness_rms(all_chains, ident, w, h):.2f} px "
          f"-> {straightness_rms(all_chains, params, w, h):.2f} px")
    for path, img in kept_imgs.items():
        before = center_vs_full(img)
        after = center_vs_full(img, params)
        print(f"{path}: center-vs-full angle "
              f"{'n/a' if before is None else f'{before:.2f} deg'} -> "
              f"{'n/a' if after is None else f'{after:.2f} deg'}")

    os.makedirs(os.path.dirname(os.path.abspath(args.yaml)), exist_ok=True)
    save_params(args.yaml, {**params,
                            "width": kept_imgs and next(iter(kept_imgs.values())).shape[1] or w,
                            "height": kept_imgs and next(iter(kept_imgs.values())).shape[0] or h})
    print(f"saved {args.yaml}")


if __name__ == "__main__":
    main()
