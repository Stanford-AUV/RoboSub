from setuptools import find_packages, setup

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
    description="This package acts as a bridge between the software and the virtual hardware on Gazebo. Takes in the same inputs as the hardware package, but instead of sending commands to the real robot, it sends commands to the virtual Gazebo robot.",
    license="TODO: License declaration",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "thrusters = simulation.nodes.thrusters:main",
            "sensors = simulation.nodes.sensors:main",
            "path_bridge = simulation.nodes.path_bridge:main",
        ],
    },
)
