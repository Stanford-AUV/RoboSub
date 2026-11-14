from setuptools import find_packages, setup
import os

package_name = "simulation"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    # bridge.py loads gazebo_bridge.yaml from its own folder, so install the
    # YAML next to it; without this, the installed bridge finds no topic list.
    package_data={"simulation": ["gazebo_bridge.yaml"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ros",
    maintainer_email="scotthickmann21@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "thrusters = simulation.nodes.thrusters:main",
            "ground_truth = simulation.nodes.ground_truth:main",
            "path_bridge = simulation.nodes.path_bridge:main",
            "imu = simulation.nodes.imu:main",
            "arduino = simulation.nodes.arduino:main",
            "oak = simulation.nodes.oak:main",
            "gz_dvl_bridge = simulation.nodes.gz_dvl_bridge:main",
            "dvl = simulation.nodes.dvl:main"
        ],
    },
)
