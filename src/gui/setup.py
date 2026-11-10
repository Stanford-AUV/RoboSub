from setuptools import setup

package_name = "gui"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ros",
    maintainer_email="scotthickmann21@gmail.com",
    description="Browser HUD for the sub: a ROS-to-WebSocket bridge plus a "
    "self-contained HTML page. Instrumentation and manual override only.",
    license="TODO: License declaration",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "bridge = gui.ros2_gui_bridge:main",
        ],
    },
)
