from setuptools import find_packages, setup

package_name = "perception"

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
    maintainer="Stanford Robosub",
    maintainer_email="selenasun02@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"perception_node = perception.nodes.perception_node:main",
            f"debug_node = perception.nodes.debug_node:main",
            f"data_node = perception.nodes.data_node:main",
            f"image_publisher = perception.nodes.dummy_video:main",
            f"yolov8_ros_node = perception.nodes.YOLONode:main",
        ],
    },
)
