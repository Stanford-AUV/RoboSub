from setuptools import find_packages, setup

package_name = "hardware"

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
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "thrusters = hardware.nodes.thrusters:main",
            "imu = hardware.nodes.imu:main",
            "dvl = hardware.nodes.dvl:main",
            "sensors = hardware.nodes.sensors:main",
            "arduino = hardware.nodes.arduino:main",
            "localization_test = hardware.nodes.localization_test:main",
            "localization_plot = hardware.nodes.localization_plot:main",
            "imu_plot = hardware.nodes.imu_plot:main",
            "dvl_plot = hardware.nodes.dvl_plot:main",
            "sensors_plot = hardware.nodes.sensors_plot:main",
        ],
    },
)
