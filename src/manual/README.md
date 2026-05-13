# Package: `manual`

**Build type:** `ament_python`  
**Role:** Manual control over **NATS**: the ROS nodes subscribe to JSON state on a NATS subject and publish wrenches (and aux topics). A **host-side** script captures physical keys or joystick and publishes to NATS.

## Executables (`ros2 run manual <name>`)

| Executable | Module | Summary |
|------------|--------|---------|
| `keyboard` | `nodes/keyboard.py` | **`KeyboardNode`:** connects to **`nats://localhost:4222`**, subscribes to subject **`keyboard`**. Decodes **`KeyboardState`** JSON (`manual.utils.keyboard`). Publishes **`geometry_msgs/WrenchStamped`** on **`wrench`** (→ **`/wrench`** in default namespace), **`std_msgs/Int16`** on **`light`**, **`torpedo`**, **`dropper`**. Force/torque magnitudes start at **0.5** and scale with **t/g/y/h/u/j** (see `KeyboardState`); capped by **MAX_FORCE** / **MAX_TORQUE** (**1.0**). On NATS timeout, sends an empty `KeyboardState` (zero motion). |
| `joystick` | `nodes/joystick.py` | Same NATS pattern on subject **`joystick`** with **`JoystickState`**. Publishes **`wrench`**, **`light`**, **`torpedo`**, **`dropper`**. Subscribes **`depth`** (`msgs/Float32Stamped`) for depth-hold when enabled. |

## Host scripts (not `ros2 run`)

| Script | Purpose |
|--------|---------|
| **`keyboard_local.py`** | **`pynput`** listener; publishes **`KeyboardState`** JSON to NATS subject **`keyboard`** every 50 ms. Key map is defined in code: **w/s** forward/back, **a/d** left/right, **r/f** up/down, **q/e** CCW/CW yaw, **t/g** overall vel scale, **y/h** linear, **u/j** angular. Run via **`./keyboard_local.sh`** from repo root (expects **`.local_venv`** per [local_requirements.txt](../../local_requirements.txt)). |
| **`joystick_local.py`** | Pygame joystick → NATS **`joystick`**. **`./joystick_local.sh`**. |

## `main/launch/manual.py` (as coded)

Starts **`control/thrust_generator`** and **`manual/keyboard`** with `global.yaml`. Requires **`nats-server`** and **`./keyboard_local.sh`**.

**Do not launch `manual.py` at the same time as `hardware.py`** — each starts its own **`thrust_generator`**. For the vehicle, use **`hardware.py`** + **`ros2 run manual keyboard`** + **`keyboard_local.sh`**. For **bench** teleop without full hardware, **`ros2 launch main manual.py`** + **`keyboard_local.sh`** is fine.

## Dependencies

- **ROS:** `geometry_msgs`, `std_msgs`, **`msgs`** (see `package.xml`).
- **Python (nodes):** `nats-py`; **`KeyboardState`** uses **`dataclasses-json`** (see workspace `requirements.txt` / devcontainer install).
- **Python (`keyboard_local.py`):** `pynput`, `nats-py`, `dataclasses-json` (see **`local_requirements.txt`** for the host venv).

## Docs

See `manual/manual.md` for a short module overview.
