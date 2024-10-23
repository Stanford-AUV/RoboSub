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

- For MacOS users [click here](#macos-installation)
- For Windows users [click here](#windows-installation)

### MacOS Installation

#### Part 1: VM Installation

1. Install VMWare Fusion (the free version, not the pro version).
2. Download the Ubuntu ISO (either AMD or ARM depending on your computer) from https://cdimage.ubuntu.com/noble/daily-live/current/.
3. Open VMWare and drag the downloaded ISO for installation.
4. Press continue, then finish, and name your instance (leaving the default name is fine).
5. Select "Try to install Ubuntu". Then click "Install Ubuntu" from the desktop.
6. Proceed with the installation leaving everything as the default values.
7. Shutdown the virtual machine (Virtual Machine > Shutdown).
8. Go to Virtual Machine > Settings > CD/DVD and select autodetect.
9. Still in settings, go to Startup Disk and select Hard Disk and then Restart.
10. When prompted for Ubuntu, press enter, and then login.
11. Run the following: `sudo apt-get update && sudo apt-get install open-vm-tools-desktop`
12. Shutdown the Virtual Machine, then restart it, and copy-pasting shortcuts should work.
13. Run the following in a Terminal: `sudo apt-get install openssh-server && sudo apt install net-tools && sudo apt install git-all && service ssh start`.
14. Follow the steps in https://gazebosim.org/docs/harmonic/install_ubuntu/#binary-installation-on-ubuntu
15. Follow steps 1 and 2 of https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository.
16. Follow steps 1 to 3 of https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user.
17. Note the USER@HOST shown in the VM's Ubuntu Terminal. Then open a Terminal on your local computer (i.e. your Mac) and enter `ssh USER@HOST.local`.
18. If you connect successfully, now type `logout`, and then type `ssh-keygen -t ed25519` leaving all fields as default. Finally type `ssh-copy-id -i ~/.ssh/id_ed25519.pub USER@HOST.local`, once more replacing USER and HOST with your own credentials.


#### MacOS Camera Installation (Optional)

To access the camera directly from within the Docker in the VM, in a Terminal on the VM (not VSCode!), run the following:
```
sudo udevadm control --reload-rules && sudo udevadm trigger
```
followed by
```
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
```

Then, connect the camera to your laptop. Make sure when prompted on the VM that the camera points to Linux and not Mac.

Contact kevinli7@stanford.edu if you encounter any issues.

#### Part 2: Code Installation

1. Download VSCode.
2. Install the Remote Development extension.
3. Open a new VSCode window.
4. Press `Control` + `Shift` + `P`.
5. Type `Remote-SSH: Connect to Host`.
6. Press `+ Add New SSH Host`.
7. Type `ssh USER@HOST.local`.
8. Press connect at the bottom right, and wait for connection to be successful.
9. From the files tab on the left, press `Clone Repository`, `Clone from GitHub`, enter the repository name `Stanford-AUV/RoboSub`, and finally enter a location to clone the repository to (something like `~/GitHub/`).
10. Open that newly cloned repository in a VSCode window.
11. When prompted to open the project in a Docker container at the bottom right, press `Reopen in Container`.
12. Wait for the build process to take place and complete.
13. Create a new VSCode Terminal (`Terminal` > `New Terminal`).
14. You should be all set! Proceed to the [building section](#building).

### Windows Installation

1. Open the Microsoft store and search up `Ubuntu 24.04`
2. Download and open from the store when the installation completes
3. Set a username and password (make sure to write it down somewhere!)
4. Follow the steps in https://gazebosim.org/docs/harmonic/install_ubuntu/#binary-installation-on-ubuntu
5. Follow steps 1 to 3 of https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository.
6. Follow steps 1 to 3 of https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user.
7. From the files tab on the left, press `Clone Repository`, `Clone from GitHub`, enter the repository name `Stanford-AUV/RoboSub`, and finally enter a location to clone the repository to (something like `~/GitHub/`).
8. Open that newly cloned repository in a VSCode window.
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

## Simulation

Since simulation requires a GUI, we will only run the simulation code from VSCode. The simulation itself will run directly on the VM.

To use the Gazebo simulator, follow these steps:

1. Make sure you have the VM window easily accessible. The Gazebo window will show up inside the VM.
2. In a Terminal on the VM (not in VSCode!), run `xhost +local:docker`
3. Back on the VSCode Terminal, run `./sim.sh`
4. A window should pop up in the VM with the simulation environment displayed. Press play to start the simulation.
5. To stop the simulation, press `Control` + `C` on the VSCode Terminal

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
