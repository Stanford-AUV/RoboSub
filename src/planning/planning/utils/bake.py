"""Offline trajectory bake: waypoint YAML -> sampled legs + pursuit descriptors.

This is the ONLY place create_path runs; path_generator just plays the baked
file, so no spline math ever happens in the water.
"""

import hashlib

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from planning.utils.create_path import create_path



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
        wp.get("publish"),
    )


def _split_legs(waypoints):
    """Split at pause>0 waypoints; the pausing waypoint seeds the next leg.
    Each leg carries the publish topic (or None) of its terminal waypoint -
    the runtime fires it once the leg's pause elapses. Same splitting logic
    as the old in-node generation."""
    legs, current = [], []
    for wp in waypoints:
        current.append(wp)
        if wp[6] > 0.0:
            legs.append((current, wp[6], wp[8]))
            current = [wp]
    if len(current) > 1 or not legs:
        legs.append((current, 0.0, wp[8]))
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


def _bake_leg(leg_wps, pause_after, publish):
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
    if publish:
        item["publish"] = str(publish)
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


def _bake_waypoint_legs(wp_tuples):
    """Bake a flat list of waypoint tuples into leg items (free-yaw + pause
    splitting applied). Shared by ordinary segments and each branch option."""
    out = []
    if len(wp_tuples) >= 2:
        for leg_wps, pause_after, publish in _split_legs(
            _substitute_free_yaws(wp_tuples)
        ):
            out.append(_bake_leg(leg_wps, pause_after, publish))
    return out


def bake(yaml_path):
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)

    items = []
    pending = []  # waypoint tuples not yet baked into legs

    def flush_pending():
        items.extend(_bake_waypoint_legs(pending))
        pending.clear()

    for key in data:
        segment = data[key]
        if segment.get("type") == "branch":
            if "decision" not in segment:
                raise ValueError(f"branch segment '{key}' needs a decision topic")
            flush_pending()
            options = []
            for opt_key in segment:
                if not opt_key.startswith("waypoints"):
                    continue
                opt = segment[opt_key]
                if "code" not in opt:
                    raise ValueError(
                        f"branch option '{key}.{opt_key}' needs a code"
                    )
                wp_tuples = [_wp_tuple(wp) for wp in opt["waypoints"]]
                options.append(
                    {"code": opt["code"], "items": _bake_waypoint_legs(wp_tuples)}
                )
            if not options:
                raise ValueError(f"branch segment '{key}' has no waypoint options")
            items.append(
                {
                    "type": "branch",
                    "decision": str(segment["decision"]),
                    "options": options,
                }
            )
        else:
            for wp in segment["waypoints"]:
                pending.append(_wp_tuple(wp))
    flush_pending()

    if not items:
        raise ValueError(f"No trajectory items produced from '{yaml_path}'")
    return {"source_sha256": source_hash(yaml_path), "items": items}


def save_bake(doc, out_path):
    with open(out_path, "w") as f:
        yaml.safe_dump(doc, f, default_flow_style=None)


def load_bake(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)


def main(argv=None):
    """CLI: bake a waypoint YAML into <name>.baked.yaml next to the source.

    Run ON SHORE (no spline math happens in the water):
        python -m planning.utils.bake src/planning/planning/segments.yaml
    """
    import sys

    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1:
        print("usage: bake.py <waypoints.yaml>")
        sys.exit(2)
    src = argv[0]
    out = (src[: -len(".yaml")] if src.endswith(".yaml") else src) + ".baked.yaml"

    # Cache: if an existing bake already matches the source hash, the splines
    # are identical - skip the recompute.
    try:
        if load_bake(out).get("source_sha256") == source_hash(src):
            print(f"{out} is up to date (source unchanged) - nothing to do")
            return
    except (OSError, FileNotFoundError, TypeError):
        pass  # missing/unreadable bake: regenerate

    doc = bake(src)
    save_bake(doc, out)
    legs = sum(1 for i in doc["items"] if i["type"] == "leg")
    branches = sum(1 for i in doc["items"] if i["type"] == "branch")
    branch_legs = sum(
        len(o["items"])
        for i in doc["items"]
        if i["type"] == "branch"
        for o in i["options"]
    )
    print(
        f"Baked {out}: {legs} leg(s), {branches} branch(es) "
        f"({branch_legs} option leg(s))"
    )


if __name__ == "__main__":
    main()