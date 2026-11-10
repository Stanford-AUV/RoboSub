import os
from glob import glob
from setuptools import find_packages, setup

package_name = "localization"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        # ekf.yaml goes to the share dir so launch files resolve it via
        # get_package_share_directory() under any install layout.
        (os.path.join("share", package_name), glob("localization/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ros",
    maintainer_email="scotthickmann21@gmail.com",
    description="State estimation. Owns the EKF configuration that fuses the hardware sensor topics and the perception heading correction into a single /odometry/filtered, plus any nodes built on top of it.",
    license="TODO: License declaration",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [],
    },
)
