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
    maintainer_email="scotthickmann21@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "perception_node = perception.nodes.perception_node:main",
            "test.video = perception.nodes.test.video:main",
            "test.view_video = perception.nodes.test.view_video:main",
            "test.view_detections_3d = perception.nodes.test.view_detections_3d:main",
            "camera = perception.nodes.camera:main",
            "objects_localizer = perception.nodes.objects_localizer:main",
        ],
    },
)
