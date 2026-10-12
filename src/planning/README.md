# `planning`

**Build type:** `ament_python` · **Depends on:** `msgs`, `nav_msgs`, `std_msgs`, `rclpy`, `control` (for `control.utils.state`)

The mission layer. A waypoint YAML is **baked offline** into sampled trajectory legs, and
at run time `path_generator` plays that bake back as a 60 Hz pose setpoint on
`/desired/pose` — which is exactly what [`control`](../control/README.md)'s `pid_control`
chases. No spline math ever runs in the water.

---

## Entry points

Registered in [`setup.py`](setup.py) — run with `ros2 run planning <name>`.

| Executable | Module | Node name |
|---|---|---|
| `path_generator` | `planning/nodes/path_generator.py` | `path_generator` |
| `path_streamer` | `planning/utils/stream_path_points.py` | `path_streamer` |

---

## Nodes

### `path_generator` — the mission player

| Direction | Topic | Type |
|---|---|---|
| Subscribes | `/odometry/filtered` | `nav_msgs/Odometry` |
| Subscribes | `<decision>` (e.g. `/pinger`) | `std_msgs/String` — only while sitting on a branch item |
| Publishes | `/desired/pose` | `nav_msgs/Odometry` (60 Hz) |
| Publishes | `/mission/stage` | `std_msgs/String` (1 Hz heartbeat of the current stage) |
| Publishes | `<decision>/listening` | `std_msgs/String` — `"start"` / `"stop"` around a branch's listen window |
| Publishes | per-waypoint `publish:` topics (e.g. `/torpedo_left`) | `std_msgs/String` |

**Parameter:** `waypoints_path` (default `segments.yaml`) — a bare filename resolves
against the package share dir.

The node **exits itself** ~3 s after the path finishes. `main/launch/planning.py` wires
`on_exit=Shutdown()` to that, which is how `launch_sub.sh` detects mission completion and
tears the whole stack down with neutral PWM.

### `path_streamer` — commanded-path plot
Subscribes `/desired/pose` (`nav_msgs/Odometry`); publishes nothing. Keeps a 3D
matplotlib trail of the commanded position. With no `DISPLAY` it falls back to `Agg` and
periodically writes `PATH_STREAMER_IMAGE` (default `/tmp/path_streamer.png`).

---

## Mission YAML

Each top-level key is a **segment**. A segment is either a plain list of waypoints (a
*leg*) or `type: branch`.

```yaml
segment_0:
  waypoints:
    - position: {x: 0.0, y: 0.0, z: 0.0}
      orientation: {roll: 0.0, pitch: 0.0, yaw: 0.0}
    - position: {x: 1.0, y: 0.0, z: -0.6}
      orientation: {roll: 0.0, pitch: 0.0, yaw: 0.0}
      pause: 6.0            # station-keep here; also splits the path into a new leg
      publish: /torpedo_left # fire-and-forget String published on arrival

segment_1:
  type: branch
  decision: /pinger          # topic whose majority String vote picks the option
  waypoints_0:
    code: front              # value the decision topic must report
    waypoints: [...]
  waypoints_1:
    code: back
    waypoints: [...]
```

**Units:** positions in metres; `roll`/`pitch`/`yaw` in degrees, fixed `xyz` order.
Both branch options should start and end at the same point.

**Available YAMLs:** `segments.yaml`, `prequal.yaml`, `semifinals.yaml`,
`semifinals_pinger.yaml`, `wild_card.yaml`, `localization_stress_test.yaml`,
`sample_path.yaml`. Each has a committed `*.baked.yaml` sibling.

### Baking is mandatory

```bash
python tools/bake_path.py src/planning/planning/segments.yaml
```

`path_generator` loads the `.baked.yaml` and verifies a hash of the source YAML
(`planning/utils/bake.py:source_hash`), so an edited-but-unbaked YAML is rejected rather
than silently stale. `launch_sub.sh` re-bakes automatically (a fresh bake is a no-op).

### Offline preview (no ROS)

```bash
python src/planning/planning/utils/visualize_path.py src/planning/planning/segments.yaml
```

Runs the same `create_path` spline used by the bake and draws speed-coloured segments with
heading arrows. Needs `numpy`, `scipy`, `matplotlib`, `pyyaml` and a `DISPLAY` (`TkAgg`).

---

## Library code (`planning/utils/`)

| Module | Contents |
|---|---|
| `bake.py` | `load_bake()`, `source_hash()` — the only place `create_path` runs |
| `create_path.py` | PCHIP spline trajectory generation through waypoints |
| `pursuit.py` | pure-pursuit stepping (`pursuit_target`, `step_toward`, `arrived`) for a planned `go_to_object` segment type. **Imported by `path_generator` but not yet reachable** — the only item types the bake and the player understand today are `leg` and `branch`. |
| `geometry.py` | `override_yaw()` — replace yaw, keep roll/pitch |
| `stream_path_points.py` | the `path_streamer` node |
| `visualize_path.py` | offline plotter |

---

## Launch

`main/launch/planning.py` ([`../main`](../main/README.md)) starts `path_generator` +
`path_streamer`:

```bash
ros2 launch main planning.py waypoints_path:=semifinals.yaml
# or, for the whole stack:
./launch_sub.sh semifinals.yaml
```

## Tests

`colcon test --packages-select planning`, or `./test.sh` from the workspace root.
