# RoboSub

## Overview

This is the repository for all code used by Stanford RoboSub's AUV.

The main components of the RoboSub tech stack are:

- **[ROS 2 Jazzy](https://docs.ros.org/en/jazzy/index.html)**: A middleware framework for robotics development, enabling distributed nodes to communicate efficiently. This project leverages ROS 2's enhanced real-time capabilities, especially for underwater navigation and sensor data processing.
  
- **Docker**: The project runs within Docker containers, ensuring environment consistency and simplifying setup.
  
- **Python/C++**: ROS nodes are primarily written in Python, though C++ can also used for performance-critical tasks.
  
- **[Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install/)**: Gazebo is used to simulate underwater dynamics, allowing developers to test control algorithms and mission planning in a virtual environment before deploying them to the actual RoboSub hardware.

- **VSCode with Remote SSH**: The development workflow relies heavily on Visual Studio Code's Remote SSH feature, enabling developers to work with identical setups. This setup allows for seamless development from local machines while leveraging the computational resources of remote servers and avoiding setup inconsistencies across the team.

## Warnings

When developing with ROS 2 and Gazebo, it's easy to suddenly be reading outdated documentation online concerning past ROS and Gazebo versions. Make sure you are always looking at sources that refer to ROS 2 Jazzy and Gazebo Harmonic.

## Installation

1. Install [Docker Desktop](https://www.docker.com/products/docker-desktop) for your operating system.
2. In a new VSCode window, clone the `Stanford-AUV/Robosub` repository:
    ```bash
    git clone https://github.com/Stanford-AUV/Robosub.git
    ```
3. Open the Command Palette (`Cmd+Shift+P`) and select `Reopen in Container`.
5. Wait for the build process to take place and complete.
6. Create a new VSCode Terminal (`Terminal` > `New Terminal` or Ctrl+Shift+`).
9. Install the Gazebo simulator by following the steps in [SIMULATION.md](/SIMULATION.md).

You should be all set! Proceed to the [building section](#building).

## Building

To build the project, in the VSCode Terminal, run:
```bash
./build.sh && source install/setup.bash
```
Please ignore `jobserver unavailable` errors.
One built, you can proceed to the [running section](#running).

## Running

### Running hardware module

Make sure to give your computer/the Orin permission to connect to the necessary external devices by running:
```bash
./ports.sh
```

### Running multiple ROS nodes at once through a launch file (preferred method)

Run:
```bash
ros2 launch src/launch/LAUNCH_FILE
```
For example:
```bash
ros2 launch src/launch/control.py
```
To launch the main launch file:
```bash
ros2 launch main main.py
```

### Running ROS nodes directly

To run a ROS node, please run the following:
```bash
ros2 run PACKAGE_NAME NODE_NAME
```
For example:
```bash
ros2 run control test_thrust
```

## Simulation

Since simulation requires a GUI, we will only run the simulation code from VSCode. The simulation itself will run directly on the VM.

To use the Gazebo simulator, follow these steps:

1. Make sure you have the VM window easily accessible. The Gazebo window will show up inside the VM.
2. In a Terminal on the VM (not in VSCode!), run `xhost +local:docker`
3. Back on the VSCode Terminal, run `./sim.sh`
4. A window should pop up in the VM with the simulation environment displayed. Press play to start the simulation.
5. To stop the simulation, press `Control` + `C` on the VSCode Terminal

## Joystick

Please run these commands in order.

### Part 1. Docker Portion

On three seperate Terminals within VSCode, run the following commands:

```bash
nats-server
ros2 launch main manual.py
ros2 run manual joystick
```

### Part 2. Local Portion

Open a Terminal window from the Terminal app (not VSCode!), and cd into this repository's root.

Then run the following command:

```bash
./joystick_local.sh
```

## Camera

To record data from the camera, run:
```bash
ros2 launch perception camera.py
```
Then once both camera show that they are ready, run:
```bash
ros2 run perception camera_viewer
```
Finally, to also record camera data, run:
```bash
./record_cameras.sh PATH_TO_RECORDING
```
To replay camera data:
```bash
ros2 bag play PATH_TO_RECORDING
```

## Viewing the ROS Graph

After launching a launch file, you can view the ROS graph of all nodes and topics currently active. To do so, run the following command in another Terminal:
```bash
ros2 run rqt_graph rqt_graph
```

## Creating New Nodes

Make sure to create new nodes inside the according package under the `nodes` directory. Make sure to also add that file in the `setup.py` file of the package.

## Testing

To run all unit tests, run:
```bash
./test.sh
```
In case a `pep257` test fails, run the following to get more details about the error:
```bash
ament_pep257
```

## Manual Control

In a first Terminal, run:
```bash
ros2 launch src/launch/manual.py
```

In a second Terminal, run:
```bash
ros2 run manual keyboard
```

## FAQ

## Display Issue

Getting a display or xcb/XQuartz issue? Follow these steps:

1. **Make sure XQuartz is running on your Mac** and that you ran `xhost +` on your Mac Terminal.

2. **Get the display value from SSH Terminal (outside Docker):**
    ```bash
    echo $DISPLAY
    ```
    Copy the result.

3. **Set the display variable in Docker Terminal where you need XQuartz:**
    ```bash
    export DISPLAY=result
    ```
    (Replace `result` with the actual value you copied from step 2)

4. Back on the Orin, run:
    ```bash
    xeyes
    ```
and you should see a GUI pop up through XQuartz. If not, check your XQuartz installation and ensure it allows remote networks.

### Out of storage

If you have less than 30 GB total on your machine, contact Scott Hickmann for how to resize to at least 30 GB. If you do have a max storage above 30 GB but still ran out of storage, make sure to clean up unused Docker files. You can do so by running the following command from the VM Terminal:
```bash
sudo docker system prune -a -f
```

### Could not load the Qt platform plugin "xcb"

If you encounter the following error:
```
Could not load the Qt platform plugin "xcb"
```
Try rebooting the VM from the VM Terminal. Then restart the Docker container and things should run again.

### Unable to connect to a sensor

1. Make sure when you connect the sensor to your computer, you select "Connect to Linux" when prompted by the VM.
2. If this still fails, run the following code within your VSCode Terminal, followed by the port of the device:
```bash
sudo chmod a+rw PORT
```
For example:
```
sudo chmod a+rw /dev/ttyACM0
```

If this concerns the IMU, make sure to run from the Orin Terminal (not the Docker container):
```bash
cd ~/GitHub/xsens_mt (or the xsens_mt directory in general)
sudo modprobe usbserial
sudo insmod ./xsens_mt.ko
```

Additionally, make sure the DVL is unplugged if running the IMU while the Orin is wall powered instead of battery powered.
