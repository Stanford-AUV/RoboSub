from setuptools import setup

setup(
    name='gui',
    version='0.0.1',
    packages=['gui'],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'bridge = gui.ros2_gui_bridge:main',
        ],
    },
)
