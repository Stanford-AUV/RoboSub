# Onboarding: mission path YAML

This project gets you comfortable with how we represent **mission segments** and **waypoints**, how to **preview** a path before running ROS, how to **stream** it through the planning stack, and how that ties into **vehicle bring-up** (keyboard teleop and autonomy).

Reference implementation: [`src/planning/planning/sample_path.yaml`](../src/planning/planning/sample_path.yaml).

---

## What you will do

1. Author your own YAML file (same schema as `sample_path.yaml`).
2. Plot it offline with `visualize_path.py` (smooth trajectory through `create_path`).
3. Run the planning ROS pipeline: `path_loader` → `path_generator` → `path_streamer` and watch the commanded pose on **`/desired/pose`**.
4. Run the **vehicle autonomy stack** (no teleop) and understand how your path connects to **`controller`** (see [Step 4](#step-4--autonomous-regime-vehicle-no-teleop)).

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
- Name your file something unique, e.g. `onboarding/paths/member_<sunet>_path.yaml`, and add it under version control when you are happy with it.

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
| `planning/path_loader` | Reads your YAML, publishes `nav_msgs/Path` on **`/waypoints`** on a timer (default **10 s** between segment advances — see `cycle_seconds` in `load_path.py`). Pass a file path after `--` (see `main` in `load_path.py`). |
| `planning/path_generator` | Subscribes **`/waypoints`**, builds a smooth trajectory, publishes **`nav_msgs/Odometry`** on **`/desired/pose`** at 60 Hz. |
| `planning/path_streamer` | Subscribes **`/desired/pose`**, maintains a **matplotlib 3D** trail of the commanded position (and optional heading arrow). |

Open **three** terminals, each with `source install/setup.bash`.

**Terminal A — load your YAML:**

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

**Display:** `path_streamer` needs a working `DISPLAY` for an interactive window. If matplotlib falls back to `Agg` (no display), it logs a warning and periodically saves **`PATH_STREAMER_IMAGE`** (default **`/tmp/path_streamer.png`**).

**Stop:** `Ctrl+C` in each terminal.

---

## Step 4 — Autonomous regime (vehicle, no teleop)

**Goal:** run your **mission geometry** through planning while the **vehicle autonomy stack** is up, **without** keyboard/joystick publishing wrenches.

### 4.1 What “fully autonomous” means in this repo

- **`ros2 launch main main.py`** starts **`hardware.py`** (including **`thrust_generator`**), **`localization.py`**, and **`main/launch/control.py`** (**`controller`** + **`test_controller`**). **`manual.py` is commented out** — teleop is not part of this launch.
- **`main/launch/control.py`** does **not** start **`thrust_generator`** (so it is not duplicated with `hardware.py` when both are included from `main.py`). See the header comment in that file if you run **`ros2 launch main control.py`** without hardware.
- **`controller`** (`control/nodes/controller.py`) subscribes to **`/odometry/filtered`** and to **`waypoint`**. The reference type is **`msgs/Waypoint`**, not `nav_msgs/Odometry`. Goals are applied when **`Waypoint.purpose`** is **`"target"`** (or **`"follow"`** with a subject); see **`reference_callback`** in that file.
- **`planning/path_generator`** publishes **`nav_msgs/Odometry`** on **`/desired/pose`**. That topic is **not** subscribed to by **`controller`** today. To drive the sub from your YAML spline you need a small **relay** (subscribe **`/desired/pose`** → publish **`msgs/Waypoint`** with `purpose="target"` and `target=<Odometry>`) or a mission node your team adds.

**Parallel workflow for onboarding:** run **`main.py`** and, in other terminals, **`path_loader`** + **`path_generator`** so **`/desired/pose`** matches your YAML; compare to **`/odometry/filtered`** in logs or Foxglove while you validate the autonomy stack.

### 4.2 Preconditions

1. **Stop teleop:** quit **`keyboard_local`**; do not run **`manual/keyboard`** or **`manual/joystick`** so **`/wrench`** is not contested by operators.
2. **`./ports.sh`**, sensors powered. **`nats-server`** is **not** required for autonomy (only for teleop).

### 4.3 Launch full autonomy

```bash
source install/setup.bash
ros2 launch main main.py
```

Confirm **`/odometry/filtered`**, **`/thrusts`**, and **`/wrench`**. **`controller`** listens for **`msgs/Waypoint`** on **`waypoint`** (relative name → **`/waypoint`** in the default namespace).

### 4.4 Run your YAML trajectory alongside (logging / validation)

```bash
ros2 run planning path_loader -- /full/path/to/your_path.yaml
ros2 run planning path_generator
# optional: ros2 run planning path_streamer
```

### 4.5 Closing the loop from `/desired/pose` to `controller`

Publish **`msgs/Waypoint`** on **`/waypoint`** with:

- **`purpose: "target"`**
- **`target`**: `nav_msgs/Odometry` (e.g. aligned with each **`/desired/pose`** update)

Use QoS compatible with **`controller`**’s subscription (see `controller.py` — **TRANSIENT_LOCAL** on the reference subscription).

### 4.6 Optional: waypoint JSON and `path_tracker`

`control/path_tracker` is meant to stream **`msgs/Waypoint`** from a **JSON** file. Example pose layout: `src/control/control/nodes/test_waypoints copy.json` (same folder as `path_tracker.py`). Confirm with your leads that the node matches your branch before pool trials.

---

## Troubleshooting

| Issue | What to try |
|-------|-------------|
| `yaml.scanner.ScannerError` / load failure | Validate indentation and keys; compare to `sample_path.yaml`. |
| `path_generator` logs errors from `create_path` | Often too few waypoints or degenerate geometry; add points or spread them out. |
| No plot window | Set `DISPLAY`, or inspect `/tmp/path_streamer.png` after a few seconds. |
| Empty `/waypoints` or no motion | Check `path_loader` logs and YAML path; restart `path_loader` to re-publish. |
| `/desired/pose` moves but sub does not follow | **`controller`** expects **`msgs/Waypoint`** on **`waypoint`**, not **`/desired/pose`**. See [§4.5](#45-closing-the-loop-from-desiredpose-to-controller). |

---

## Where to read more

- [Step 4 — Autonomous regime (this file)](#step-4--autonomous-regime-vehicle-no-teleop)
- [Hardware sensor preflight (pool day)](preflight_sensors.md)
- [Planning package README](../src/planning/README.md)
- [Root README](../README.md)

When you finish, consider opening a PR with your YAML under [`onboarding/paths/`](paths/) so leads can review geometry and naming.

---

## Robot bring-up: hardware, keyboard teleop, autonomy

This section matches **`main/launch`**, **`manual`**, and **`control`** as in the repo today.

### Keyboard teleop (NATS)

- ROS node **`manual/keyboard`** (`manual/nodes/keyboard.py`) connects to **`nats://localhost:4222`**, subscribes to subject **`keyboard`**, and publishes **`WrenchStamped`** on **`wrench`** (→ **`/wrench`**), plus **`light`**, **`torpedo`**, **`dropper`**.
- Host script **`manual/keyboard_local.py`** (run via **`./keyboard_local.sh`** from repo root) uses **`pynput`** and publishes **`KeyboardState`** JSON to NATS. Dependencies: **`.local_venv`** + **`local_requirements.txt`** (`pynput`, `nats-py`, `dataclasses-json`).

Key map in **`keyboard_local.py`**: **w/s** forward/back, **a/d** strafe, **r/f** up/down, **q/e** yaw CCW/CW, **t/g** overall scale, **y/h** linear, **u/j** angular.

### 0. Prerequisites

- **`./build.sh && source install/setup.bash`**
- **`robot_localization`** for `localization.py` / `main.py`
- **`xsens_mti_ros2_driver`** if you use stock `hardware.py` (or comment out that node)
- **`./ports.sh`** for `/dev/ttyUSB*`, `/dev/ttyACM*`, Teensy path

### 1. Power and physical checks

Power per team procedure; verify USB devices exist.

### 2. What the launch files start

| Launch | Contents (high level) |
|--------|------------------------|
| **`main main.py`** | **`hardware.py`** + **`localization.py`** + **`control.py`** ( **`controller`** + **`test_controller`**; **`thrust_generator`** only from hardware). **`manual.py` not included.** |
| **`main hardware.py`** | Xsens → **`hardware/imu`**, **`dvl`**, **`control/thrust_generator`**, **`thrusters`**, **`arduino`**; optional `sensors_plot` with `plot:=true`. |
| **`main localization.py`** | **`hardware/sensors`** + **`robot_localization/ekf_node`** + static TFs. |
| **`main manual.py`** | **`thrust_generator`** + **`manual/keyboard`**. Needs **NATS** + **`./keyboard_local.sh`**. **Do not run together with `hardware.py`** (two **`thrust_generator`** instances). |

### 3. Phase A — Keyboard teleop (pool / vehicle)

1. **`./ports.sh`**
2. **`nats-server`** (reachable at **`localhost:4222`** from ROS and from the machine running keys)
3. **`ros2 launch main hardware.py`**
4. **`ros2 run manual keyboard`**
5. From repo root: **`./keyboard_local.sh`**
6. Check **`/wrench`**, **`/thrusts`**, **`/arduino/sensors`**, **`/odometry/filtered`** if EKF is running

### 4. Phase B — Autonomy (no teleop)

- Quit **`keyboard_local`**; do not run **`manual/keyboard`** or joystick.
- **`ros2 launch main main.py`**
- For YAML-driven goals on the vehicle, see **[Step 4](#step-4--autonomous-regime-vehicle-no-teleop)** (relay from **`/desired/pose`** to **`msgs/Waypoint`**).
- **`ros2 launch main control.py` alone** does not start **`thrust_generator`**; pair with **`hardware.py`** or run **`ros2 run control thrust_generator`** (see `main/launch/control.py` comments).

### 5. Data path (teleop)

```text
keyboard_local (pynput) --NATS "keyboard"-->  manual/keyboard  --/wrench-->  thrust_generator  --/thrusts-->  thrusters  --pwms-->  arduino  -->  vehicle
```
