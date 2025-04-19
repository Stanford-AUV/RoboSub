#!/bin/bash
set -e

colcon build \
        --merge-install \
        --symlink-install \
        --cmake-args \
                -DCMAKE_BUILD_TYPE=RelWithDebInfo \
                -Wall -Wextra -Wpedantic \
        --paths src/simulation/simulation/custom_gz_plugins \
        --base-paths .
