#!/bin/bash
set -e

colcon build \
        --merge-install \
        --symlink-install \
        --cmake-args \
                -DCMAKE_BUILD_TYPE=RelWithDebInfo \
                -Wall -Wextra -Wpedantic \
        --base-paths .

# colcon COPIES setup.py data_files (all our yamls) into the install space even
# with --symlink-install, so a fresh build leaves stale yaml copies until the
# next launch self-heals. Re-symlink them now, so live source yamls are served
# immediately after every build.
"$(dirname "${BASH_SOURCE[0]}")/tools/symlink_yamls.sh"
