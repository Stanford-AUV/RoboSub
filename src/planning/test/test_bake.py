import os

import numpy as np
import pytest

from planning.utils.bake import bake, load_bake, save_bake, source_hash

SPLINE_ONLY = """
segment_0:
  waypoints:
    - position: {x: 0.0, y: 0.0, z: 0.0}
      orientation: {roll: 0.0, pitch: 0.0, yaw: 0.0}
    - position: {x: 1.0, y: 0.0, z: -0.5}
      orientation: {roll: 0.0, pitch: 0.0, yaw: 0.0}
      pause: 2.0
    - position: {x: 2.0, y: 0.0, z: -0.5}
      orientation: {roll: 0.0, pitch: 0.0, yaw: 0.0}
"""

WITH_PURSUIT = """
segment_0:
  waypoints:
    - position: {x: 0.0, y: 0.0, z: 0.0}
      orientation: {roll: 0.0, pitch: 0.0, yaw: 0.0}
    - position: {x: 1.0, y: 0.0, z: -0.5}
      orientation: {roll: 0.0, pitch: 0.0, yaw: 0.0}
segment_1:
  type: go_to_object
  object_id: gate
  standoff: 1.0
  timeout: 20.0
  exit:
    position: {x: 3.0, y: 0.0, z: -0.5}
    orientation: {roll: 0.0, pitch: 0.0, yaw: 0.0}
segment_2:
  waypoints:
    - position: {x: 5.0, y: 0.0, z: -0.5}
      orientation: {roll: 0.0, pitch: 0.0, yaw: 0.0}
"""


def _write(tmp_path, text):
    p = os.path.join(tmp_path, "wps.yaml")
    with open(p, "w") as f:
        f.write(text)
    return p


def test_bake_spline_only_splits_legs_at_pause(tmp_path):
    doc = bake(_write(str(tmp_path), SPLINE_ONLY))
    legs = [i for i in doc["items"] if i["type"] == "leg"]
    assert len(legs) == 2
    assert legs[0]["pause_after"] == 2.0
    assert legs[1]["pause_after"] == 0.0
    # 100 samples of 7 (pose) and 6 (twist)
    assert len(legs[0]["poses"]) == 100
    assert len(legs[0]["poses"][0]) == 7
    assert len(legs[0]["twists"][0]) == 6
    # Leg ends at rest
    assert np.allclose(legs[0]["twists"][-1][:3], 0.0)


def test_bake_pursuit_descriptor_and_exit_leg(tmp_path):
    doc = bake(_write(str(tmp_path), WITH_PURSUIT))
    types = [i["type"] for i in doc["items"]]
    assert types == ["leg", "go_to_object", "leg"]
    pursuit = doc["items"][1]
    assert pursuit["object_id"] == "gate"
    assert pursuit["standoff"] == 1.0
    assert pursuit["timeout"] == 20.0
    assert pursuit["arrive_tol"] == 0.25  # default
    assert pursuit["fallback"] == pursuit["exit"]  # fallback defaults to exit
    # The leg after the pursuit starts at the declared exit waypoint.
    first_pose = doc["items"][2]["poses"][0]
    assert np.allclose(first_pose[:3], [3.0, 0.0, -0.5], atol=1e-6)


def test_bake_requires_object_id_and_exit(tmp_path):
    bad = """
segment_0:
  type: go_to_object
  standoff: 1.0
"""
    with pytest.raises(ValueError):
        bake(_write(str(tmp_path), bad))


def test_save_load_roundtrip_and_hash(tmp_path):
    src = _write(str(tmp_path), SPLINE_ONLY)
    doc = bake(src)
    out = os.path.join(str(tmp_path), "wps.baked.yaml")
    save_bake(doc, out)
    loaded = load_bake(out)
    assert loaded["source_sha256"] == source_hash(src)
    assert loaded["items"][0]["type"] == "leg"
    assert np.allclose(loaded["items"][0]["poses"], doc["items"][0]["poses"])
