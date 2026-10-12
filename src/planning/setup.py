import os
from glob import glob
from setuptools import find_packages, setup

package_name = "planning"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Install the waypoint YAMLs into the package share dir so path_generator
        # can find them via get_package_share_directory() under any install
        # layout (symlink or merge/copy) - it no longer relies on __file__.
        (os.path.join("share", package_name), glob("planning/*.yaml")),
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
            "path_generator = planning.nodes.path_generator:main",
            "path_streamer = planning.utils.stream_path_points:main",
        ],
    },
)
