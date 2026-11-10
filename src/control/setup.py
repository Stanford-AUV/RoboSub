import os
from glob import glob
from setuptools import find_packages, setup

package_name = "control"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        # pid_control reads pid.yaml at startup; install it to the share dir so
        # the node works from the install tree (not just --symlink-install).
        (os.path.join("share", package_name), ["control/pid.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ros",
    maintainer_email="scotthickmann21@gmail.com",
    description="This package is in charge of controlling the robot. Given a path and the robot's state, it is able to output the magnitudes for each thruster for how the robot is supposed to move.",
    license="TODO: License declaration",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "thrust_generator = control.nodes.thrust_generator:main",
            "controller = control.nodes.controller:main",
            "pid_control = control.nodes.pid_control:main",
            "pid_visualizer = control.nodes.pid_visualizer:main",
        ]
    },
)
