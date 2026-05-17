# Package: `planning`

**Build type:** `ament_python`  
**Role:** Waypoint paths from YAML, trajectory generation along waypoints, and optional visualization of `/desired/pose`.

## Executables (`ros2 run planning <name>`)

| Executable | Module | Summary |
|------------|--------|---------|
| `path_loader` | `nodes/load_path.py` | Publishes **`/waypoints`** (`nav_msgs/Path`). **YAML file:** first non-ROS arg after `--`, else `planning/sample_path.yaml`. Example: `ros2 run planning path_loader -- /path/to/mission.yaml`. Timer `cycle_seconds` (default 10) advances segments. |
| `path_generator` | `nodes/path_generator.py` | Sub: **`/waypoints`**. Builds a smooth trajectory via `planning.utils.create_path.create_path`. Publishes **`/desired/pose`** (`nav_msgs/Odometry`) at 60 Hz by indexing into the generated trajectory. Builds internal `msgs/GeneratedPath` but **does not** publish it to a ROS topic (only in-memory for sampling). |
| `path_streamer` | `utils/stream_path_points.py` | Sub: **`/desired/pose`**. 3D matplotlib trail (`PathStreamer`); headless mode writes `PATH_STREAMER_IMAGE` or `/tmp/path_streamer.png`. Depends on `control.utils.state`. |

## Utilities (not entry points)

- `planning/utils/create_path.py` — spline-like trajectory generation used by `path_generator`.
- `planning/utils/visualize_path.py` — plotting helper (run as script / import).

## Typical graph (when used)

```text
path_loader  --/waypoints-->  path_generator  --/desired/pose-->  (pid_control or logger)
                                     |
                              path_streamer (optional plot)
```

`control/path_tracker` uses the **`get_waypoints`** service, not `/waypoints` — there is no `GetWaypoints` server in this package today.

## Data

- `planning/sample_path.yaml` — example multi-segment path for `path_loader`.
