# RoboSub

## Overview

This is the repository for all code used by Stanford RoboSub's AUV.

The main components of the RoboSub tech stack are:

- **[ROS 2 Jazzy](https://docs.ros.org/en/jazzy/index.html)**: A middleware framework for robotics development, enabling distributed nodes to communicate efficiently. This project leverages ROS 2's enhanced real-time capabilities, especially for underwater navigation and sensor data processing.
  
- **Docker**: The project runs within Docker containers, ensuring environment consistency and simplifying setup.
  
- **Python/C++**: ROS nodes are primarily written in Python, though C++ can also used for performance-critical tasks.
  
- **[Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install/)**: Gazebo is used to simulate underwater dynamics, allowing developers to test control algorithms and mission planning in a virtual environment before deploying them to the actual RoboSub hardware.

- **VSCode with Remote SSH**: The development workflow relies heavily on Visual Studio Code's Remote SSH feature, enabling developers to work on identical AWS EC2 instances. This setup allows for seamless development from local machines while leveraging the computational resources of remote servers and avoiding setup inconsistencies across the team.

## Warnings

When developing with ROS 2 and Gazebo, it's easy to suddenly be reading outdated documentation online concerning past ROS and Gazebo versions. Make sure you are always looking at sources that refer to ROS 2 Jazzy and Gazebo Harmonic.

## Installation

1. Download VSCode.
2. Install the Remote Development extension.
3. Ask Scott (@Scott Hickmann on Slack) for credentials to an AWS EC2 instance and to be added to the GitHub. You should receive two pieces of information:
   - A domain name (e.g. `ec2-35-173-183-51.compute-1.amazonaws.com`)
   - An `.pem` file. Make sure to store this in a permanent location on your computer where it will not be moved or deleted.
4. Open a new VSCode window.
5. Press `Control` + `Shift` + `P`.
6. Type `Remote-SSH: Connect to Host`.
7. Press `+ Add New SSH Host`.
8. Make note of the AWS domain and path to the `.pem` file stored in step 3, and type `ssh -i PATH_TO_PEM_FILE AWS_DOMAIN` and press enter.
9. From the files tab on the left, press `Clone Repository`, `Clone from GitHub`, enter the repository name `Stanford-AUV/RoboSub`, and finally enter a location to clone the repository to on the AWS EC2 instance.
10. Open that newly cloned repository in a VSCode window.
11. When prompted to open the project in a Docker container at the bottom right, press `Reopen in Container`.
12. Wait for the build process to take place and complete.
13. Create a new VSCode Terminal (`Terminal` > `New Terminal`).
14. You should be all set! Proceed to the [building section](#building).

## Building

To build the project, in the VSCode Terminal, run:
```bash
./build.sh && source install/setup.bash
```
One built, you can proceed to the [running section](#running).

## Running

### Running multiple ROS nodes at once through a launch file (preferred method)

Run:
```bash
ros2 launch src/launch/LAUNCH_FILE
```
For example:
```bash
ros2 launch src/launch/control.py
```

### Running ROS nodes directly

To run a ROS node, please run the following:
```bash
ros2 run PACKAGE_NAME NODE_NAME
```
For example:
```bash
ros2 run control thrust_generator
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
