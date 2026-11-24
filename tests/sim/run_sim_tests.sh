#!/bin/bash
# Run every sim test in this folder, keep going if one fails, and exit 1 if any failed.
# CI's sim-mission job calls this. To run it yourself, in the container from /ws:
#   colcon build --merge-install --symlink-install --base-paths . && source install/setup.bash
#   bash tests/sim/run_sim_tests.sh
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."

status=0
for test_file in tests/sim/test_*.py; do
    echo "=================== ${test_file}"
    launch_test "${test_file}" || status=1
done
exit "${status}"
