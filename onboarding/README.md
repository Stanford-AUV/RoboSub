# Onboarding: mission path YAML

This project gets you comfortable with how we represent **mission segments** and **waypoints**, how to **preview** a path before running ROS, and how to **stream** it through the planning stack for a live 3D visualization.

Reference implementation: [`src/planning/planning/sample_path.yaml`](../src/planning/planning/sample_path.yaml).

---

## What you will do

1. Author your own YAML file (same schema as `sample_path.yaml`).
2. Plot it offline with `visualize_path.py` (smooth trajectory through `create_path`).
3. Optionally run the ROS pipeline: `path_loader` → `path_generator` → `path_streamer` and watch the commanded pose evolve in 3D.

---

## YAML format (required fields)

Top level: one or more **segments**. Each segment is a mapping (key name is arbitrary, e.g. `segment_0`, `my_leg`).

Every segment **must** include a `waypoints` list. Other keys in `sample_path.yaml` (`concurrent_action`, `with_timeout_s`, `exit_on_reach`) are **not** read by `path_loader` or `path_generator` today; you can keep them for documentation or future mission logic.

Each waypoint:

```yaml
position:
  x: 0.0   # metres
  y: 0.0
  z: 0.0
orientation:
  roll: 0.0    # degrees
  pitch: 0.0
  yaw: 0.0
```

**Units:** positions in **metres**; Euler angles in **degrees** (`roll`, `pitch`, `yaw` about fixed `xyz` order, same as `scipy.spatial.transform.Rotation.from_euler("xyz", ..., degrees=True)` used in code).

**Tips:**

- Use at least **two** waypoints per segment if you want a smooth interpolated path (`create_path` needs a line to follow). A single waypoint is valid for plotting as a point only.
- Keep paths in a sensible volume for the sub (start small, e.g. a 1 m square in the horizontal plane before adding depth changes).
- Name your file something unique, e.g. `onboarding/member_<sunet>_path.yaml`, and add it under version control when you are happy with it.

---

## Step 1 — Offline visualization (no ROS)

This runs the same `create_path` spline logic used by `path_generator` and draws speed-coloured segments plus forward direction arrows.

**Requirements:** Python 3 with `numpy`, `scipy`, `matplotlib`, `pyyaml`. A display backend (e.g. `TkAgg`) if you want an interactive window; over SSH use X11 forwarding or run locally.

From the **repository root**:

```bash
cd src/planning/planning/utils
python3 visualize_path.py /absolute/or/relative/path/to/your_path.yaml
```

With no argument, it loads `../sample_path.yaml`.

**What to check:** waypoints appear where you expect, headings look reasonable, segments are coloured distinctly, and speed colouring along each segment has no obvious spikes (unless you intend sharp motion).

---

## Step 2 — Build and source the workspace

From the **workspace root** (directory that contains `build.sh`):

```bash
./build.sh && source install/setup.bash
```

---

## Step 3 — Live ROS visualization (`/desired/pose`)

These three nodes form a small pipeline:

| Node | Role |
|------|------|
| `planning/path_loader` | Reads your YAML, publishes `nav_msgs/Path` on **`/waypoints`** on a timer (default **10 s** between segment advances — see `cycle_seconds` in `load_path.py`). |
| `planning/path_generator` | Subscribes **`/waypoints`**, builds a smooth trajectory, publishes **`nav_msgs/Odometry`** on **`/desired/pose`** at 60 Hz. |
| `planning/path_streamer` | Subscribes **`/desired/pose`**, maintains a **matplotlib 3D** trail of the commanded position (and optional heading arrow). |

Open **three** terminals, each with `source install/setup.bash`.

**Terminal A — load your YAML** (path after `--` is passed to the node; default is `sample_path.yaml` if omitted):

```bash
ros2 run planning path_loader -- /full/path/to/your_path.yaml
```

**Terminal B — trajectory generator:**

```bash
ros2 run planning path_generator
```

**Terminal C — 3D plot:**

```bash
ros2 run planning path_streamer
```

**Display:** `path_streamer` needs a working `DISPLAY` for an interactive window. If matplotlib falls back to `Agg` (no display), it logs a warning and periodically saves **`PATH_STREAMER_IMAGE`** (default **`/tmp/path_streamer.png`**). Open that file to confirm motion.

**Stop:** `Ctrl+C` in each terminal, in reverse order if you like (streamer first is fine).

---

## Troubleshooting

| Issue | What to try |
|-------|-------------|
| `yaml.scanner.ScannerError` / load failure | Validate indentation and keys; compare to `sample_path.yaml`. |
| `path_generator` logs errors from `create_path` | Often too few waypoints or duplicate / degenerate geometry; add points or spread them out. |
| No plot window | Set `DISPLAY`, run on machine with a monitor, or inspect `/tmp/path_streamer.png` after a few seconds. |
| Empty `/waypoints` or no motion | Check Terminal A logs; confirm YAML path exists; ensure `path_loader` started before or with `path_generator` (re-publish by restarting loader if needed). |

---

## Where to read more

- [Planning package README](../src/planning/README.md) — all planning executables and topics.
- [Root README](../README.md) — workspace build and architecture.

When you finish, consider opening a PR with your YAML under e.g. `onboarding/paths/` so leads can review geometry and naming.

---

## Robot bring-up: hardware, keyboard teleop, autonomy

This section is derived from **`main/launch`**, **`hardware`**, **`control`**, and **`manual`** sources (not from sparse READMEs). On-robot teleop is **keyboard** (`manual/nodes/keyboard.py`). It publishes `geometry_msgs/WrenchStamped` on topic **`wrench`**, which resolves to **`/wrench`** in the default namespace — the same topic **`control/thrust_generator`** subscribes to.

### 0. Prerequisites on the vehicle computer

- Workspace built and sourced: `./build.sh && source install/setup.bash`.
- **`robot_localization`** installed if you use `localization.py` / `main.py` (e.g. `sudo apt install ros-jazzy-robot-localization`).
- **`xsens_mti_ros2_driver`** in the workspace if you use `hardware.py` as written (it calls `get_package_share_directory("xsens_mti_ros2_driver")`). If that package is missing, comment out the Xsens `Node` in `main/launch/hardware.py` or you cannot launch that file.
- USB/serial devices present: `hardware/arduino.py` opens **`/dev/ttyACM0`**; `hardware/thrusters.py` expects **`/dev/ttyUSB_teensy`** (see `ports.sh` list). Run **`./ports.sh`** from the repo root so those nodes can open ports without permission errors.

### 1. Power and physical checks

- Power the vehicle per team procedure; verify ESCs / Teensy / pressure board are alive.
- Confirm USB connections so the expected `/dev/ttyUSB*` and `/dev/ttyACM*` nodes exist (`ls /dev/ttyUSB* /dev/ttyACM*`).

### 2. Data path (what the launch files actually start)

**`ros2 launch main hardware.py`** starts, in order: Xsens node → `hardware/imu` (sub **`/imu/data`**) → `hardware/dvl` → **`control/thrust_generator`** → **`hardware/thrusters`** (sub **`/thrusts`**, pub relative `pwms`, sub `sensors`) → **`hardware/arduino`** (sub `pwms`, pub **`/arduino/sensors`**). Optional `sensors_plot` if `plot:=true`.

**`ros2 launch main localization.py`** starts: **`hardware/sensors`** (sub `msgs/DVLData` on topic **`dvl`**, pubs `/dvl/twist_sync`, `/depth/pose_sync`) + **`robot_localization/ekf_node`** with params from `global.yaml` + static TFs. **Note:** the current `hardware/dvl` node publishes `Twist`/`Pose` on `/velocity` and `/position`, not `DVLData` on `dvl`, so the EKF stack as wired may not receive DVL the way `sensors` expects until that is reconciled in code — still run it if your branch fixes this or you use bag replay.

**`ros2 launch main manual.py`** starts only **`control/thrust_generator`** with `global.yaml`. It does **not** start keyboard inside the launch file because **`keyboard` must run in an interactive shell** (it uses `termios` on `stdin`).

### 3. Phase A — Manual control with **keyboard** (recommended first swim)

Use **two terminals**, both `source install/setup.bash`.

1. **Actuation + sensors (and IMU path if you use `hardware.py`):**

   ```bash
   ros2 launch main hardware.py
   ```

   (Or a reduced launch your team maintains if Xsens is omitted.)

2. **Interactive keyboard teleop** (must be a real TTY, e.g. SSH session with `-t`, or local console on the Orbot — **not** a non-interactive `launch` child):

   ```bash
   ros2 run manual keyboard
   ```

   Keys are printed by the node (from `keyboard.py`): **w/a/s/d** planar forces, **r/f** ±z, **q/e** yaw torque, **t/g/y/h/u/j** scale forces; any other key zeroes motion; **Ctrl+C** quits and publishes a zero wrench.

3. **Verify data flow:** `ros2 topic echo /thrusts`, `ros2 topic echo /arduino/sensors`, and if running localization, `ros2 topic echo /odometry/filtered`.

You do **not** need NATS or `joystick_local.sh` for keyboard teleop (`joystick` node is separate).

### 4. Phase B — Autonomy (`control` stack)

**`ros2 launch main control.py`** starts **`thrust_generator`** + **`controller`** + **`test_controller`**. `controller` subscribes to **`/odometry/filtered`** and relative topic **`waypoint`** (`nav_msgs/Odometry`); `test_controller` publishes sample **`waypoint`** messages so the loop closes without a mission server.

**Important:** Do **not** run two copies of **`thrust_generator`** on the same topics. If `hardware.py` is already running, it already includes `thrust_generator` — then either stop hardware’s thrust node (not exposed as a separate launch today) or run autonomy from a launch that omits the duplicate. Practically: for a first autonomy test, stop the manual/hardware stack and run **`ros2 launch main control.py`** **only if** thrusters and odometry are still provided by other means, or adjust launches so a single `thrust_generator` serves both. Coordinate with leads; **`main.py` currently includes `hardware.py` and `manual.py`**, and `hardware.py` already starts `thrust_generator`, so **`main.py` + enabling `control.py` would duplicate `thrust_generator`** until launch files are refactored.

### 5. Quick reference — topic chain for manual drive

```text
keyboard  --/wrench-->  thrust_generator  --/thrusts-->  thrusters  --pwms-->  arduino  --(serial)-->  vehicle
```
