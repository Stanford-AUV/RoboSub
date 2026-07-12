import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(
    0,
    os.path.join(os.path.dirname(__file__), "..", "..", "src", "perception"),
)
from fit_undistortion import build_chains, fit_params, straightness_rms
from perception.utils.undistort import distort_pts, to_norm, to_px

TRUE = {"k1": -0.10, "k2": 0.02, "sy": 1.06, "width": 960, "height": 540}
W, H = 960, 540


def synthetic_chains(rng):
    """Straight world lines, sampled and pushed through the TRUE distortion."""
    chains = []
    for _ in range(14):
        p0 = rng.uniform([0, 0], [W, H])
        ang = rng.uniform(0, np.pi)
        d = np.array([np.cos(ang), np.sin(ang)])
        ts = np.linspace(-400, 400, 15)
        pts = p0 + ts[:, None] * d
        keep = (pts[:, 0] > 5) & (pts[:, 0] < W - 5) & (pts[:, 1] > 5) & (pts[:, 1] < H - 5)
        pts = pts[keep]
        if len(pts) < 6:
            continue
        dist = to_px(
            distort_pts(to_norm(pts, W, H), TRUE["k1"], TRUE["k2"], TRUE["sy"]),
            W,
            H,
        )
        dist += rng.normal(0, 0.3, dist.shape)  # detection noise
        chains.append(dist)
    return chains


def test_fit_recovers_synthetic_params():
    rng = np.random.default_rng(1)
    chains = synthetic_chains(rng)
    fitted = fit_params({"synth": chains}, W, H)
    assert abs(fitted["k1"] - TRUE["k1"]) < 0.02
    assert abs(fitted["sy"] - TRUE["sy"]) < 0.04
    # the real acceptance criterion: chains are straight after undistort
    ident = {"k1": 0.0, "k2": 0.0, "sy": 1.0, "width": W, "height": H}
    assert straightness_rms(chains, fitted, W, H) < 0.6
    assert straightness_rms(chains, fitted, W, H) < 0.25 * straightness_rms(
        chains, ident, W, H
    )


def test_build_chains_links_collinear_segments():
    # one bent "line" as 4 nearly-collinear segments + 1 far-away segment
    segs = np.array(
        [
            [100, 100, 200, 110],
            [205, 111, 300, 124],
            [305, 125, 400, 141],
            [405, 142, 500, 160],
            [700, 400, 750, 300],
        ],
        dtype=float,
    )
    chains = build_chains(segs)
    assert len(chains) == 1          # lone segment can't form a >=3 chain
    assert len(chains[0]) == 8       # 4 segments x 2 endpoints
