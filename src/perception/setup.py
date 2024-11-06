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
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"perception_node = perception.nodes.perception_node:main",
            f"test.video = perception.nodes.test.video:main",
            f"test.view_video = perception.nodes.test.view_video:main",
            f"camera = perception.nodes.camera:main",
            f"objects_detector = perception.nodes.objects_detector:main",
        ],
    },
)
