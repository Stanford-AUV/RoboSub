import os
from glob import glob
from setuptools import find_packages, setup

package_name = "perception"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name), glob("perception/cameras.yaml")),
    ],
    install_requires=[
        "setuptools",
        "torch",
        "ultralytics",
    ],
    zip_safe=True,
    maintainer="Stanford Robosub",
    maintainer_email="scotthickmann21@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "realsense_node = perception.nodes.realsense:main",
            "test.video = perception.nodes.test.video:main",
            "test.view_video = perception.nodes.test.view_video:main",
            "camera_viewer = perception.nodes.camera_viewer:main",
            "oak_node = perception.nodes.oak:main",
            "object_detection = perception.nodes.object_detections:main",
            "aligned_depth_publisher = perception.nodes.aligned_depth_publisher:main",
            "object_localizer = perception.nodes.object_localizer:main",
        ],
    },
)
