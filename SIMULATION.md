# Gazebo

Gazebo is a state of the art simulator for robotics which includes many hydrodynamic packages.

## Installation

To install the Gazebo simulator, please follow the following steps:

```bash
brew tap osrf/simulation
brew install gz-harmonic
```

It will likely fail to install ogre1.9.

Hence open Formula, find all gz-rendering* and comment out the line with depends_on "ogre1.9"

Download Xcode 16.2 from the Apple Developer Downloads page (16.3 fails to install ogre2.3), and then run:
```
sudo xcode-select -s /Applications/Xcode.app
xcodebuild -version
```
Ensure 16.2 is printed.

## Post Installation

Launch the Gazebo server in one terminal:
```bash
gz sim -v 4 shapes.sdf -s
```
Launch the Gazebo client in a seperate terminal:
```bash
gz sim -v 4 -g
```
You should see a Gazebo window with a scene containing various shapes.

## Running

To Launch our Gazebo world, run:
```bash
export GZ_SIM_RESOURCE_PATH=`pwd`/src/simulation/simulation/models:$GZ_SIM_RESOURCE_PATH
gz sim -v 4 -s src/simulation/simulation/models/world.urdf
```
and in another terminal:
```bash
gz sim -v 4 -g
```