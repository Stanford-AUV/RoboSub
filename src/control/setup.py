from setuptools import find_packages, setup

package_name = "control"

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
            "thrust_generator = control.nodes.thrust_generator:main",
            "controller = control.nodes.controller:main",
            "path_tracker = control.nodes.path_tracker:main",
            "test_path = control.nodes.test_path:main",
            "logger = control.nodes.logger:main",
            "sim_tester = control.nodes.sim_tester:main",
            "test_thrust = control.nodes.test_thrust:main",
        ]
    },
)
