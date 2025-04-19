#!/bin/bash
set -e
# To ignore the setuptools deprecation warning
export PYTHONWARNINGS="ignore::setuptools.SetuptoolsDeprecationWarning"

colcon build \
        --merge-install \
        --symlink-install \
        --cmake-args \
                -DCMAKE_BUILD_TYPE=RelWithDebInfo \
                -DCMAKE_EXPORT_COMPILE_COMMANDS=On \
                -Wall -Wextra -Wpedantic \
        --paths src/simulation/simulation/custom_gz_plugins \
        --base-paths . \
        --packages-skip xsens_mti_ros2_driver custom_gz_plugins \
        2> >(grep -v "install_name_tool")



