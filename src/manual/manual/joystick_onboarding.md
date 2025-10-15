# Software Onboarding (2025-2026) - Joystick Control

Hello! By the end of this exercise, you will have built up manual control via joystick, all the while scouring through the codebase and passively learning how code controls the sub.

## _**Overview**_
**These are the following files you will modify:**

`src/manual/manual/utils/joystick.py`
`src/manual/manual/joystick_local.py`
`src/manual/manual/nodes/joystick.py`

Note how the directory, not just file name, determines function. The connection scheme looks a bit like:

Joystick
    ↕         (USB Connection)
Computer
    ⇳         (Ethernet/Wi-Fi Connection)
Jetson Orin
    ↕         (Arduino)
Hardware

We will conquer this task top to bottom.

**Part 1: `src/manual/manual/utils/joystick.py`**  
This file defines the class which defines the JoystickState class. Its purpose is to condense the real-life configuration of the joystick controller (knob and switch positions) into data (floats, ints, bools, etc.).

Keep note of this, you may complete this now or in-tandem with the next part.

**Part 2: `src/manual/manual/joystick_local.py`**  
This file does the following:
- Recognizes and connects to the joystick controller
- Reads the configuation of the controller
- Reconstructs a JoystickState and sends through NATS server (see https://docs.nats.io/nats-concepts/what-is-nats)

Now with your JoystickState class, connect to the controller, probe which configuration states change when you modulate each knob/switch.

**Part 3: `src/manual/manual/nodes/joystick.py`**  
This file is the bulk of your work, the joystick node which houses the main manual-control logic. Whereas the previous two files operate on your local computer, this one will run on the sub. By taking the JoystickState received from the NATS server, it is now your job to:
- Understand the different publishers included in the node (what can the joystick control?). Preferrably, stick to just thrusters and lights.
- Familiarizing yourself with ROS2 filesystem and node structures, both manual and other packages
- Figuring out how all nodes work together. Specifically, each node subscribes and publishes to topics; how do topics get passed around?

All in all, as you complete your code, you should run the following to test:

On your computer:
1. Create a `.local_venv` via `python -m venv .local_venv` and `pythom -m pip install -r local_requirements.txt`
2. Run `./joystick_local.sh`

On the Orin:
1. Run `nats-server` to open the connection
2. `ros2 launch src/main/launch/hardware.py` starts up the arduino, thrusters, lights, etc.
3. `ros2 run manual joystick` runs the joystick node

Let me know if there are any issues!