import os

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

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


FREE_YAW_PREFIX = """
segment_0:
  waypoints:
    - position: {x: 0.0, y: 0.0, z: 0.0}
      orientation: {roll: 0.0, pitch: 0.0, yaw: free}
    - position: {x: 0.0, y: 0.0, z: -0.6}
      orientation: {roll: 0.0, pitch: 0.0, yaw: 90.0}
    - position: {x: 2.0, y: 0.0, z: -0.6}
      orientation: {roll: 0.0, pitch: 0.0, yaw: 90.0}
"""


def test_bake_free_yaw_mask_covers_first_interval(tmp_path):
    doc = bake(_write(str(tmp_path), FREE_YAW_PREFIX))
    leg = doc["items"][0]
    mask = leg["yaw_free"]
    assert len(mask) == len(leg["poses"])
    assert mask[0] == 1
    assert mask[-1] == 0
    # Free until the first controlled waypoint, contiguous prefix.
    first_controlled = mask.index(0)
    assert all(v == 1 for v in mask[:first_controlled])
    assert all(v == 0 for v in mask[first_controlled:])
    # Free yaws are baked flat at the next controlled yaw (90 deg).
    yaws = [
        Rotation.from_quat(p[3:]).as_euler("xyz", degrees=True)[2]
        for p in leg["poses"]
    ]
    assert np.allclose(yaws, 90.0, atol=1e-6)


def test_bake_all_waypoints_free(tmp_path):
    text = FREE_YAW_PREFIX.replace("yaw: 90.0", "yaw: free")
    doc = bake(_write(str(tmp_path), text))
    assert all(v == 1 for v in doc["items"][0]["yaw_free"])


def test_bake_no_free_yaw_omits_mask(tmp_path):
    doc = bake(_write(str(tmp_path), SPLINE_ONLY))
    assert all("yaw_free" not in i for i in doc["items"])


def test_bake_rejects_bad_yaw_string(tmp_path):
    bad = SPLINE_ONLY.replace("yaw: 0.0", "yaw: loose", 1)
    with pytest.raises(ValueError):
        bake(_write(str(tmp_path), bad))


WITH_BRANCH = """
segment_0:
  waypoints:
    - position: {x: 0.0, y: 0.0, z: -1.0}
      orientation: {roll: 0.0, pitch: 0.0, yaw: 0.0}
    - position: {x: 2.0, y: 0.0, z: -1.0}
      orientation: {roll: 0.0, pitch: 0.0, yaw: 0.0}
segment_1:
  type: branch
  topic: /pinger/task
  listen_sec: 4.0
  default: torpedo_first
  branches:
    torpedo_first:
      torpedo:
        type: go_to_object
        object_id: torpedo_target
        standoff: 1.0
        timeout: 30.0
        exit:
          position: {x: 4.0, y: 1.0, z: -1.0}
          orientation: {roll: 0.0, pitch: 0.0, yaw: 0.0}
      octagon:
        waypoints:
          - position: {x: 6.0, y: 0.0, z: -1.0}
            orientation: {roll: 0.0, pitch: 0.0, yaw: 0.0}
    octagon_first:
      octagon:
        waypoints:
          - position: {x: 5.0, y: -1.0, z: -1.0}
            orientation: {roll: 0.0, pitch: 0.0, yaw: 0.0}
          - position: {x: 6.0, y: 0.0, z: -1.0}
            orientation: {roll: 0.0, pitch: 0.0, yaw: 0.0}
  exit:
    position: {x: 6.0, y: 0.0, z: -1.0}
    orientation: {roll: 0.0, pitch: 0.0, yaw: 0.0}
segment_2:
  waypoints:
    - position: {x: 8.0, y: 0.0, z: -1.0}
      orientation: {roll: 0.0, pitch: 0.0, yaw: 0.0}
"""


def test_bake_branch_item_shape_and_both_branches(tmp_path):
    doc = bake(_write(str(tmp_path), WITH_BRANCH))
    types = [i["type"] for i in doc["items"]]
    assert types == ["leg", "branch", "leg"]
    br = doc["items"][1]
    assert br["topic"] == "/pinger/task"
    assert br["listen_sec"] == 4.0
    assert br["default"] == "torpedo_first"
    assert set(br["branches"]) == {"torpedo_first", "octagon_first"}
    # torpedo_first: pursuit descriptor then a leg to the octagon point
    tf = br["branches"]["torpedo_first"]
    assert [i["type"] for i in tf] == ["go_to_object", "leg"]
    assert tf[0]["object_id"] == "torpedo_target"
    # octagon_first: pure spline legs
    of = br["branches"]["octagon_first"]
    assert all(i["type"] == "leg" for i in of)


def test_bake_branch_seeding_and_exit(tmp_path):
    doc = bake(_write(str(tmp_path), WITH_BRANCH))
    br = doc["items"][1]
    # Branch legs are seeded at the last pre-branch waypoint (2,0,-1).
    of_first_pose = br["branches"]["octagon_first"][0]["poses"][0]
    assert np.allclose(of_first_pose[:3], [2.0, 0.0, -1.0], atol=1e-6)
    # Both branches end at the declared exit (6,0,-1).
    for name in ("torpedo_first", "octagon_first"):
        last_pose = br["branches"][name][-1]["poses"][-1]
        assert np.allclose(last_pose[:3], [6.0, 0.0, -1.0], atol=1e-6)
    # The post-branch leg starts at the exit, like pursuit exit seeding.
    after = doc["items"][2]["poses"][0]
    assert np.allclose(after[:3], [6.0, 0.0, -1.0], atol=1e-6)


def test_bake_branch_validation_errors(tmp_path):
    # default not in branches
    bad = WITH_BRANCH.replace("default: torpedo_first", "default: nope")
    with pytest.raises(ValueError):
        bake(_write(str(tmp_path), bad))
    # missing listen_sec
    bad = WITH_BRANCH.replace("  listen_sec: 4.0\n", "")
    with pytest.raises(ValueError):
        bake(_write(str(tmp_path), bad))
    # branch not ending at the declared exit
    bad = WITH_BRANCH.replace(
        """      octagon:
        waypoints:
          - position: {x: 5.0, y: -1.0, z: -1.0}
            orientation: {roll: 0.0, pitch: 0.0, yaw: 0.0}
          - position: {x: 6.0, y: 0.0, z: -1.0}
            orientation: {roll: 0.0, pitch: 0.0, yaw: 0.0}
""",
        """      octagon:
        waypoints:
          - position: {x: 5.0, y: -1.0, z: -2.0}
            orientation: {roll: 0.0, pitch: 0.0, yaw: 0.0}
""",
    )
    with pytest.raises(ValueError):
        bake(_write(str(tmp_path), bad))


def test_bake_rejects_nested_branch(tmp_path):
    nested = """
segment_0:
  waypoints:
    - position: {x: 0.0, y: 0.0, z: -1.0}
      orientation: {roll: 0.0, pitch: 0.0, yaw: 0.0}
    - position: {x: 1.0, y: 0.0, z: -1.0}
      orientation: {roll: 0.0, pitch: 0.0, yaw: 0.0}
segment_1:
  type: branch
  topic: /pinger/task
  listen_sec: 3.0
  default: a
  branches:
    a:
      inner:
        type: branch
        topic: /x
        listen_sec: 1.0
        default: c
        branches: {c: {s: {waypoints: []}}}
        exit:
          position: {x: 2.0, y: 0.0, z: -1.0}
          orientation: {roll: 0.0, pitch: 0.0, yaw: 0.0}
    b:
      s:
        waypoints:
          - position: {x: 2.0, y: 0.0, z: -1.0}
            orientation: {roll: 0.0, pitch: 0.0, yaw: 0.0}
  exit:
    position: {x: 2.0, y: 0.0, z: -1.0}
    orientation: {roll: 0.0, pitch: 0.0, yaw: 0.0}
"""
    with pytest.raises(ValueError):
        bake(_write(str(tmp_path), nested))
