# Manual control package

**Maintainers:** see repository contacts (e.g. Scott Hickmann — `manual.md` template).

## Purpose

ROS nodes and host helpers for **operator control** of the AUV: wrench commands to `thrust_generator`, lights, and future actuators.

## Architecture (current code)

- **`manual/nodes/keyboard.py`** — **`KeyboardNode`**: subscribes to **NATS** subject **`keyboard`**, deserializes **`KeyboardState`** (`manual/utils/keyboard.py`), publishes **`WrenchStamped`** on **`wrench`**, and **`Int16`** on **`light`**, **`torpedo`**, **`dropper`**.
- **`manual/keyboard_local.py`** — runs on a machine with a keyboard; uses **`pynput`** to track keys and **publishes** JSON to NATS **`keyboard`** (same broker URL as the node: **`nats://localhost:4222`** by default).
- **`manual/nodes/joystick.py`** — same idea with subject **`joystick`** and **`joystick_local.py`** / **`joystick_local.sh`**.

Inputs are no longer read from ROS node `stdin`; teleop requires **NATS + host script** (or another NATS client).

## Outputs

- **`wrench`** → consumed by **`control/thrust_generator`** as **`/wrench`** when namespaces align.
- **`light`**, **`torpedo`**, **`dropper`** → consumed by **`hardware/arduino`** (relative topics; remap if you use namespaces).

## References

Keyboard host bridge pattern mirrors the joystick path; legacy Brown / OSRF keyboard `termios` sample has been replaced by the NATS-based stack in this repo.
