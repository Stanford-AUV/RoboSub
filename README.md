# RoboSub

## Installation

TODO

## Building

To build the project, run:
```bash
./build.sh && source install/setup.bash
```

## Development

### Running multiple ROS nodes at once through a launch file (preferred method)

First make sure you are in the launch directory:
```bash
cd src/launch
```
Then run:
```bash
ros2 launch LAUNCH_FILE
```
For example:
```bash
ros2 launch control.py
```

### Running ROS nodes directly

To run a ROS node, please run the following:
```bash
ros2 run PACKAGE_NAME NODE_NAME
```
For example:
```bash
ros2 run control thrusters
```

## Tasks

To work on this project, there are various VSCode tasks you should use. You can access those tasks by typing `Control` + `Shift` + `P` and then typing and selecting `Tasks: Run Task`. Important tasks are shown here in detail:

### setup

Use this task to update ROS and your container (equivalent to running `./setup.sh`). You only need to run this occasionally.

### build

Use this task to build the project (equivalent to running `./build.sh`). You must run this whenever you first use the project, add/remove nodes or packages, import third party packages, etc.

Make sure to run the following afterwards:
```bash
source install/setup.sh
```

### test

Use this task to run all unit tests (equivalent to running `./test.sh`).

### new ament_python package

Use this task to create a new package in the `src` folder.

## Messages

Unlike the previous version of the sub, we no longer have a global messages package. All messages should be owned by the respective most relevant package.