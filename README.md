# RoboSub

## Installation

TODO

## Tasks

To work on this project, there are various VSCode tasks you should use. You can access those tasks by typing `Control` + `Shift` + `P` and then typing and selecting `Tasks: Run Task`. Important tasks are shown here in detail:

### setup

Use this task to update Ros and your container. You only need to run this occasionally.

### build

Use this task to build the project. You must run this whenever you first use the project, add/remove nodes or packages, import third party packages, etc.

Make sure to run the following afterwards:
```bash
source install/setup.sh
```

### new ament_python package

Use this task to create a new package in the `src` folder.

## Messages

Unlike the previous version of the sub, we no longer have a global messages package. All messages should be owned by the respective most relevant package.