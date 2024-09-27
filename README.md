# RoboSub

## Installation

TODO

## Building

To build the project, run:
```bash
./build.sh && source install/setup.bash
```

## Running

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