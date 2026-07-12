import os
from glob import glob
from setuptools import find_packages, setup

package_name = "hardware"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Config YAMLs (sensors.yaml, thrusters.yaml) go to the share dir so
        # nodes resolve them via get_package_share_directory() under any
        # install layout - __file__-relative paths break on copy installs.
        (os.path.join("share", package_name), glob("hardware/*.yaml")),
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
            "thrusters = hardware.nodes.thrusters:main",
            "imu = hardware.nodes.imu:main",
            "bno085_0 = hardware.nodes.bno085:main_0",
            "bno085_1 = hardware.nodes.bno085:main_1",
            "imu_plot_orientation = hardware.nodes.imu_plot_orientation:main",
            "dvl = hardware.nodes.dvl:main",
            "sensors = hardware.nodes.sensors:main",
            "arduino = hardware.nodes.arduino:main",
            "depth = hardware.nodes.depth_sensor:main",
            "localization_test = hardware.nodes.localization_test:main",
            "localization_plot = hardware.nodes.localization_plot:main",
            "imu_plot = hardware.nodes.imu_plot:main",
            "dvl_plot = hardware.nodes.dvl_plot:main",
            "sensors_plot = hardware.nodes.sensors_plot:main",
            "ekf_watchdog = hardware.nodes.ekf_watchdog:main",
        ],
    },
)
