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
| **joystick (host)** | Moved out of this repo to the standalone **`robosub_local/`** folder (`~/Documents/repos/robosub_local`) so the pilot laptop needs no ROS. Pygame joystick → NATS **`joystick`**; run with `./run.sh --host <sub-ip>`. See the section below. |

## Joystick teleop over the tether (laptop pilot → sub)

Full pilot-a-real-sub procedure. The controller plugs into the **pilot laptop**;
the sub runs `nats-server` and the ROS node.

**On the sub (Orin):** one command from repo root —
```
./teleop_remote.sh
```
It activates the `robosub` env, sources the workspace, and brings up the whole
teleop stack: `hardware.py` (thrust_generator + thrusters + arduino/light +
depth + dvl), `nats-server` (binds `0.0.0.0:4222`), and the `manual/joystick`
node. On Ctrl+C it kills the hardware group and drives neutral PWM (1500) to all
thrusters so nothing latches (same safety as `launch_sub.sh`). Node output goes
to `logs/teleop_*/` (`tail -f` to watch). Options: `-p wrench_scale:=1.5`
(softer push), `NO_HARDWARE=1` (hardware already running elsewhere), `PLOT=1`
(sensors_plot GUI). Do **not** run it alongside `launch_sub.sh`/`main.py`.

**On the pilot laptop** — uses the standalone **`robosub_local/`** repo
(`~/Documents/repos/robosub_local`), no ROS required:
1. Find the sub's tether IP on the sub with `ip -brief addr` (tether subnet is
   `192.168.2.0/24`; the sub was `192.168.2.2`).
2. First time only — identify your controller's axes/buttons:
   `./run.sh --discover` (wiggle sticks / press buttons, note the `aN` / `bN`
   numbers, edit the `AXES` / `BUTTONS` map at the top of `joystick_local.py`).
3. Fly it: `./run.sh --host 192.168.2.2` (or just `./run.sh`, which defaults to
   `SUB_IP=192.168.2.2`). The script auto-creates `.venv` on first run.

**Control map (default XInput layout):**

| Input | Action |
|-------|--------|
| **Hold RB / R1 (button 5)** | **Deadman — sub only moves while held; release = instant stop** |
| Left stick | surge (up=forward) / sway (left-right) |
| Right stick X | yaw |
| Right stick Y | depth (up=ascend), with PD depth-hold |
| A / cross (button 0) | cycle lights: off → half → full |

Safety: on NATS timeout **or** deadman released, the node publishes a zero
wrench, so losing the tether / laptop / controller stops the sub rather than
latching the last command.

## `main/launch/manual.py` (as coded)

Starts **`control/thrust_generator`** and **`manual/keyboard`**. Requires **`nats-server`** and **`./keyboard_local.sh`**.

**Do not launch `manual.py` at the same time as `hardware.py`** — each starts its own **`thrust_generator`**. For the vehicle, use **`hardware.py`** + **`ros2 run manual keyboard`** + **`keyboard_local.sh`**. For **bench** teleop without full hardware, **`ros2 launch main manual.py`** + **`keyboard_local.sh`** is fine.

## Dependencies

- **ROS:** `geometry_msgs`, `std_msgs`, **`msgs`** (see `package.xml`).
- **Python (nodes):** `nats-py`; **`KeyboardState`** uses **`dataclasses-json`** (see workspace `requirements.txt` / devcontainer install).
- **Python (`keyboard_local.py`):** `pynput`, `nats-py`, `dataclasses-json` (see **`local_requirements.txt`** for the host venv).

## Docs

See `manual/manual.md` for a short module overview.
