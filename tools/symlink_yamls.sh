#!/usr/bin/env bash
#
# Re-symlink every config/waypoint YAML from src/ into the install space.
#
# Why: colcon --symlink-install symlinks ament_python package dirs but COPIES
# setup.py data_files (all our yamls). Every rebuild therefore silently turns
# the install-space yamls back into stale copies, and edits to src yamls never
# reach the running stack (07/12: the robot ran an old prequal.yaml this way).
# Running this after every build — and automatically from launch_sub.sh —
# guarantees the install space always serves the live source files.
#
# Usage: ./tools/symlink_yamls.sh   (idempotent, safe to run any time)
set -u

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
INSTALL_SHARE="${REPO_DIR}/install/share"

# src_dir:install_subdir pairs — mirror each package's setup.py data_files.
MAPPINGS=(
    "src/planning/planning:planning"
    "src/hardware/hardware:hardware"
    "src/perception/perception:perception"
    "src/control/control:control"
    "src/main/launch/params:main/launch/params"
)

linked=0
for m in "${MAPPINGS[@]}"; do
    src_dir="${REPO_DIR}/${m%%:*}"
    dst_dir="${INSTALL_SHARE}/${m##*:}"
    [ -d "$src_dir" ] || continue
    mkdir -p "$dst_dir"
    for f in "$src_dir"/*.yaml; do
        [ -e "$f" ] || continue
        dst="${dst_dir}/$(basename "$f")"
        # -n: if dst is already a symlink, replace it rather than descending.
        ln -sfn "$f" "$dst"
        linked=$((linked + 1))
    done
done

echo "symlink_yamls: ${linked} yamls symlinked into ${INSTALL_SHARE}"

# Fail loudly if any own-package yaml in the install space is still a copy.
bad=0
for m in "${MAPPINGS[@]}"; do
    dst_dir="${INSTALL_SHARE}/${m##*:}"
    [ -d "$dst_dir" ] || continue
    for f in "$dst_dir"/*.yaml; do
        [ -e "$f" ] || continue
        if [ ! -L "$f" ]; then
            echo "symlink_yamls: WARNING: $f is a COPY, not a symlink" >&2
            bad=1
        fi
    done
done
exit "$bad"
