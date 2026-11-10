import os
from glob import glob
from setuptools import find_packages, setup

package_name = "planning"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Waypoint YAMLs go to the share dir so path_generator finds them via
        # get_package_share_directory() under any install layout.
        (
            os.path.join("share", package_name, "paths", "missions"),
            glob("planning/paths/missions/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "paths", "test"),
            glob("planning/paths/test/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "paths", "scratch"),
            glob("planning/paths/scratch/*.yaml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ros",
    maintainer_email="scotthickmann21@gmail.com",
    description="This package is in charge of deciding what the robot should do next based on information about the state of the robot. It should output a desired path for the robot to take.",
    license="TODO: License declaration",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "path_generator = planning.nodes.path_generator:main",
            "path_streamer = planning.utils.stream_path_points:main",
        ],
    },
)
