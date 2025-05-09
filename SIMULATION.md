# Gazebo

Gazebo is a state of the art simulator for robotics which includes many hydrodynamic packages.

## Installation

### Part 1. Gazebo Installation on MacOS

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

#### Verify the Installation

Launch the Gazebo server in one terminal:

```bash
gz sim -v 4 shapes.sdf -s
```

Launch the Gazebo client in a seperate terminal:

```bash
gz sim -v 4 -g
```

You should see a Gazebo window with a scene containing various shapes.

### Part 2. Docker <-> Local Bridge Installation

Open a Terminal window from the Terminal app (not VSCode!), and cd into this repository's root.

Then run the following commands:

```bash
/opt/homebrew/bin/python3 -m venv .local_venv --system-site-packages
source .local_venv/bin/activate
pip install -r local_requirements.txt
deactivate
```

## Running

Please run these commands in order.

## Part 1. Docker Portion

On a Terminal within VSCode, run the following command:

```bash
./sim_docker.sh
```

## Part 2. Local Portion

Open a Terminal window from the Terminal app (not VSCode!), and cd into this repository's root.

Then run the following command:

```bash
./sim_local.sh
```
