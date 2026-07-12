import os
import sys

import cv2
import numpy as np

sys.path.insert(
    0,
    os.path.join(os.path.dirname(__file__), "..", "..", "src", "perception"),
)
from perception.utils.undistort import (
    build_remap,
    distort_pts,
    load_params,
    save_params,
    to_norm,
    to_px,
    undistort_pts,
)

PARAMS = {"k1": -0.12, "k2": 0.03, "sy": 1.08, "width": 1920, "height": 1080}


def test_norm_roundtrip():
    pts = np.array([[0.0, 0.0], [1919.0, 1079.0], [960.0, 540.0]])
    back = to_px(to_norm(pts, 1920, 1080), 1920, 1080)
    assert np.allclose(back, pts, atol=1e-9)
    # center maps to origin
    assert np.allclose(to_norm(np.array([[960.0, 540.0]]), 1920, 1080), 0.0)


def test_distort_inverts_undistort():
    rng = np.random.default_rng(0)
    pts = rng.uniform(-0.7, 0.7, size=(200, 2))
    und = undistort_pts(pts, PARAMS["k1"], PARAMS["k2"], PARAMS["sy"])
    back = distort_pts(und, PARAMS["k1"], PARAMS["k2"], PARAMS["sy"])
    assert np.allclose(back, pts, atol=1e-6)


def test_identity_params_are_noop():
    pts = np.array([[0.3, -0.5], [-0.6, 0.2]])
    assert np.allclose(undistort_pts(pts, 0.0, 0.0, 1.0), pts)
    assert np.allclose(distort_pts(pts, 0.0, 0.0, 1.0), pts)


def test_remap_straightens_synthetic_line(tmp_path):
    # Draw a straight line, distort the image with the model, then remap
    # with build_remap and check the line is straight again.
    w, h = 640, 360
    img = np.zeros((h, w), np.uint8)
    cv2.line(img, (40, 60), (600, 300), 255, 3)

    # Warp "clean -> distorted": each distorted pixel samples the clean
    # image at its undistorted location.
    ys, xs = np.mgrid[0:h, 0:w].astype(np.float64)
    pn = to_norm(np.stack([xs.ravel(), ys.ravel()], axis=1), w, h)
    un = undistort_pts(pn, PARAMS["k1"], PARAMS["k2"], PARAMS["sy"])
    up = to_px(un, w, h)
    dist_img = cv2.remap(
        img,
        up[:, 0].reshape(h, w).astype(np.float32),
        up[:, 1].reshape(h, w).astype(np.float32),
        cv2.INTER_LINEAR,
    )

    map_x, map_y = build_remap(w, h, PARAMS)
    fixed = cv2.remap(dist_img, map_x, map_y, cv2.INTER_LINEAR)

    def max_line_residual(image):
        pts = np.argwhere(image > 128)[:, ::-1].astype(np.float64)  # x,y
        pts -= pts.mean(axis=0)
        _, _, vt = np.linalg.svd(pts, full_matrices=False)
        return np.abs(pts @ vt[1]).max()

    assert max_line_residual(dist_img) > 3.0   # distortion visibly bent it
    assert max_line_residual(fixed) < 1.5      # remap straightened it


def test_params_yaml_roundtrip(tmp_path):
    path = str(tmp_path / "u.yaml")
    save_params(path, PARAMS)
    loaded = load_params(path)
    for k, v in PARAMS.items():
        assert abs(loaded[k] - v) < 1e-12 if isinstance(v, float) else loaded[k] == v
    assert load_params(str(tmp_path / "missing.yaml")) is None
