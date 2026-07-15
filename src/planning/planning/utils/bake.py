"""Offline trajectory bake: waypoint YAML -> sampled legs + pursuit descriptors.

This is the ONLY place create_path runs; path_generator just plays the baked
file, so no spline math ever happens in the water.
"""

import hashlib

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from planning.utils.create_path import create_path

PURSUIT_DEFAULTS = {"standoff": 0.0, "timeout": 30.0, "arrive_tol": 0.25}


def source_hash(yaml_path):
    with open(yaml_path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def _wp_tuple(wp):
    yaw = wp["orientation"]["yaw"]
    yaw_free = isinstance(yaw, str)
    if yaw_free and yaw != "free":
        raise ValueError(f"orientation.yaw must be a number or 'free', got {yaw!r}")
    return (
        wp["position"]["x"],
        wp["position"]["y"],
        wp["position"]["z"],
        wp["orientation"]["roll"],
        wp["orientation"]["pitch"],
        0.0 if yaw_free else yaw,
        float(wp.get("pause", 0.0)),
        yaw_free,
    )


def _split_legs(waypoints):
    """Split at pause>0 waypoints; the pausing waypoint seeds the next leg.
    Same logic as the old in-node generation."""
    legs, current = [], []
    for wp in waypoints:
        current.append(wp)
        if wp[6] > 0.0:
            legs.append((current, wp[6]))
            current = [wp]
    if len(current) > 1 or not legs:
        legs.append((current, 0.0))
    return legs


def _substitute_free_yaws(waypoints):
    """Bake each free yaw flat at the NEXT controlled waypoint's yaw (0.0 if
    the path never regains control), so the spline is continuous through the
    free region; the runtime mask overrides these samples anyway."""
    wps = [list(wp) for wp in waypoints]
    next_yaw = 0.0
    for wp in reversed(wps):
        if wp[7]:
            wp[5] = next_yaw
        else:
            next_yaw = wp[5]
    return [tuple(wp) for wp in wps]


def _bake_leg(leg_wps, pause_after):
    arrays = [np.array([wp[i] for wp in leg_wps]) for i in range(6)]
    (
        positions,
        velocities,
        _acc,
        orientations,
        angular_velocities,
        _aacc,
        duration,
        knots,
    ) = create_path(*arrays)
    quats = Rotation.from_euler("xyz", orientations, degrees=True).as_quat()
    poses = [
        [
            float(positions[0][i]),
            float(positions[1][i]),
            float(positions[2][i]),
            float(quats[i][0]),
            float(quats[i][1]),
            float(quats[i][2]),
            float(quats[i][3]),
        ]
        for i in range(len(positions[0]))
    ]
    twists = [
        [
            float(velocities[0][i]),
            float(velocities[1][i]),
            float(velocities[2][i]),
            float(angular_velocities[i][0]),
            float(angular_velocities[i][1]),
            float(angular_velocities[i][2]),
        ]
        for i in range(len(positions[0]))
    ]
    item = {
        "type": "leg",
        "duration": float(duration),
        "pause_after": float(pause_after),
        "poses": poses,
        "twists": twists,
    }
    flags = [bool(wp[7]) for wp in leg_wps]
    if any(flags):
        # A sample is yaw-free iff the waypoint interval it falls in starts
        # at a free waypoint (samples are uniform in [0, duration]; knots
        # are the per-waypoint times).
        t_fine = np.linspace(0.0, float(duration), len(poses))
        idx = np.clip(
            np.searchsorted(knots, t_fine, side="right") - 1, 0, len(flags) - 1
        )
        item["yaw_free"] = [int(flags[j]) for j in idx]
    return item


def _wp_list(wp):
    return [
        float(wp["position"]["x"]),
        float(wp["position"]["y"]),
        float(wp["position"]["z"]),
        float(wp["orientation"]["roll"]),
        float(wp["orientation"]["pitch"]),
        float(wp["orientation"]["yaw"]),
    ]


BRANCH_REQUIRED = ("topic", "listen_sec", "default", "branches", "exit")


def _flush_pending(pending, items):
    if len(pending) >= 2:
        for leg_wps, pause_after in _split_legs(_substitute_free_yaws(pending)):
            items.append(_bake_leg(leg_wps, pause_after))
    pending.clear()


def _bake_go_to_object(key, segment, items, pending):
    if "object_id" not in segment or "exit" not in segment:
        raise ValueError(f"go_to_object segment '{key}' needs object_id and exit")
    _flush_pending(pending, items)
    exit_wp = _wp_list(segment["exit"])
    fallback = (
        _wp_list(segment["fallback"]) if "fallback" in segment else list(exit_wp)
    )
    items.append(
        {
            "type": "go_to_object",
            "object_id": str(segment["object_id"]),
            "standoff": float(
                segment.get("standoff", PURSUIT_DEFAULTS["standoff"])
            ),
            "timeout": float(segment.get("timeout", PURSUIT_DEFAULTS["timeout"])),
            "arrive_tol": float(
                segment.get("arrive_tol", PURSUIT_DEFAULTS["arrive_tol"])
            ),
            "fallback": fallback,
            "exit": exit_wp,
        }
    )
    # The next spline leg starts at the declared exit waypoint - baked
    # offline, independent of wherever pursuit actually ends.
    pending.append(tuple(exit_wp) + (0.0, False))


def _bake_branch(key, segment, items, pending):
    missing = [k for k in BRANCH_REQUIRED if k not in segment]
    if missing:
        raise ValueError(f"branch segment '{key}' missing {missing}")
    branches = segment["branches"]
    if not isinstance(branches, dict) or len(branches) < 2:
        raise ValueError(f"branch segment '{key}' needs >=2 named branches")
    if segment["default"] not in branches:
        raise ValueError(
            f"branch segment '{key}' default '{segment['default']}' "
            f"is not one of {sorted(branches)}"
        )
    exit_wp = _wp_list(segment["exit"])
    # Seed each branch with the last pre-branch waypoint so its first leg
    # splines continuously from the listening pose.
    seed = pending[-1] if pending else None
    _flush_pending(pending, items)
    baked_branches = {}
    for name, seg_map in branches.items():
        b_items, b_pending = [], []
        if seed is not None:
            b_pending.append(seed)
        _bake_segments(seg_map, b_items, b_pending, allow_branch=False)
        end = b_pending[-1] if b_pending else None
        _flush_pending(b_pending, b_items)
        if not b_items:
            raise ValueError(f"branch '{key}.{name}' produced no items")
        if end is None or not all(
            abs(end[i] - exit_wp[i]) <= 1e-6 for i in range(6)
        ):
            raise ValueError(
                f"branch '{key}.{name}' must end at the declared exit "
                f"waypoint {exit_wp}, got {end}"
            )
        baked_branches[name] = b_items
    items.append(
        {
            "type": "branch",
            "topic": str(segment["topic"]),
            "listen_sec": float(segment["listen_sec"]),
            "default": str(segment["default"]),
            "branches": baked_branches,
        }
    )
    # Post-branch legs spline from the declared exit, independent of the
    # branch taken (both branches are validated to end there).
    pending.append(tuple(exit_wp) + (0.0, False))


def _bake_segments(data, items, pending, allow_branch=True):
    for key, segment in data.items():
        kind = segment.get("type") if isinstance(segment, dict) else None
        if kind == "go_to_object":
            _bake_go_to_object(key, segment, items, pending)
        elif kind == "branch":
            if not allow_branch:
                raise ValueError(f"nested branch segment '{key}' is not allowed")
            _bake_branch(key, segment, items, pending)
        else:
            for wp in segment["waypoints"]:
                pending.append(_wp_tuple(wp))


def bake(yaml_path):
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)
    items, pending = [], []
    _bake_segments(data, items, pending)
    _flush_pending(pending, items)
    if not items:
        raise ValueError(f"No trajectory items produced from '{yaml_path}'")
    return {"source_sha256": source_hash(yaml_path), "items": items}


def save_bake(doc, out_path):
    with open(out_path, "w") as f:
        yaml.safe_dump(doc, f, default_flow_style=None)


def load_bake(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)
