"""
Visualize the generated 3D path from a YAML waypoint file.

Usage:
    python3 visualize_path.py [path/to/file.yaml]

Defaults to planning/sample_path.yaml if no argument is given.
"""

import sys
import os
import yaml
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from scipy.spatial.transform import Rotation

from create_path import create_path


def load_yaml(yaml_path):
    with open(yaml_path) as f:
        data = yaml.safe_load(f)
    segments = []
    for key in data:
        segment = data[key]
        if "waypoints" in segment:
            segments.append(segment["waypoints"])
    return segments


def parse_waypoints(waypoints):
    positions, eulers = [], []
    for wp in waypoints:
        positions.append([wp["position"]["x"], wp["position"]["y"], wp["position"]["z"]])
        eulers.append([wp["orientation"]["roll"], wp["orientation"]["pitch"], wp["orientation"]["yaw"]])
    return np.array(positions).T, np.array(eulers).T  # both shape (3, n)


def plot_segments(segments, yaml_path):
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection="3d")
    colors = plt.cm.tab10.colors

    for seg_idx, waypoints in enumerate(segments):
        color = colors[seg_idx % len(colors)]
        positions_in, eulers_in = parse_waypoints(waypoints)

        if positions_in.shape[1] < 2:
            # Single waypoint — just plot it
            ax.scatter(*positions_in[:, 0], s=80, color=color, marker="*", zorder=5)
            continue

        pos, vel, _, ori, _, _, _ = create_path(*positions_in, *eulers_in)
        x, y, z = pos

        # Speed as colour along the path
        speed = np.sqrt(vel[0]**2 + vel[1]**2 + vel[2]**2)
        speed_norm = (speed - speed.min()) / (speed.max() - speed.min() + 1e-9)

        # Draw path coloured by speed
        for i in range(len(x) - 1):
            ax.plot(x[i:i+2], y[i:i+2], z[i:i+2],
                    color=plt.cm.plasma(speed_norm[i]), linewidth=2)

        # Orientation arrows every ~10 points
        step = max(1, len(x) // 10)
        for i in range(0, len(x), step):
            # Forward vector from yaw/pitch/roll
            fwd = Rotation.from_euler("xyz", ori[i], degrees=True).apply([0.05, 0, 0])
            ax.quiver(x[i], y[i], z[i], fwd[0], fwd[1], fwd[2],
                      color=color, length=0.1, normalize=True, alpha=0.7)

        # Waypoints
        wx, wy, wz = positions_in
        ax.scatter(wx, wy, wz, s=60, color=color, edgecolors="black",
                   zorder=5, label=f"segment_{seg_idx} waypoints")
        for i, (px, py, pz) in enumerate(zip(wx, wy, wz)):
            ax.text(px, py, pz, f" {i}", fontsize=8, color=color)

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title(f"Path: {os.path.basename(yaml_path)}\n(colour = speed, arrows = forward direction)")
    ax.legend()

    sm = plt.cm.ScalarMappable(cmap="plasma")
    sm.set_array([])
    fig.colorbar(sm, ax=ax, shrink=0.5, label="Speed (normalised)")

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.realpath(__file__))
    default_yaml = os.path.abspath(os.path.join(script_dir, "..", "sample_path.yaml"))
    yaml_path = sys.argv[1] if len(sys.argv) > 1 else default_yaml

    print(f"Loading: {yaml_path}")
    segments = load_yaml(yaml_path)
    plot_segments(segments, yaml_path)

