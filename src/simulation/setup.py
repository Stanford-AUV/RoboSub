from setuptools import find_packages, setup
import os

package_name = "simulation"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
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
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "thrusters = simulation.nodes.thrusters:main",
            "sensors = simulation.nodes.sensors:main",
            "dvl_bridge = simulation.nodes.dvl_bridge:main",
            "path_bridge = simulation.nodes.path_bridge:main",
        ],
    },
)
