#!/usr/bin/env bash
# Generate the full-map Scan Context descriptor DB (.scdb) for a venue PCD.
# The initial_align node (init_mode=auto, the default) loads <map>.scdb at
# startup to localize the drone anywhere in the map without an initial pose.
#
# Usage:
#   ./build_map_scdb.sh MAP.pcd                  # grid 1.0m, lidar height 0.5m
#   ./build_map_scdb.sh MAP.pcd 1.0 0.5          # explicit grid / lidar height
# Output: MAP.pcd.scdb next to the map file.
#
# Requires the fast_lio package to be built (./build.sh lite). The tool binary
# lives in devel/lib/fast_lio/build_map_scdb after the build.
set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCDB_ARGS=("$@")
# ROS/catkin setup scripts inspect positional arguments. Do not let our CLI
# flags leak into them; restore the arguments before parsing below.
set --
source /opt/ros/noetic/setup.bash
source "${WS_DIR}/devel/setup.bash"
set -- "${SCDB_ARGS[@]}"
set -euo pipefail

if (( $# < 1 )); then
    echo "usage: $0 MAP.pcd [grid_step_m=1.0] [lidar_height_m=0.5]" >&2
    exit 2
fi

MAP_VALUE="$1"
if [[ ! -f "${MAP_VALUE}" ]]; then
    echo "ERROR: map PCD not found: ${MAP_VALUE}" >&2
    exit 3
fi
OUT_VALUE="${MAP_VALUE}.scdb"
shift

exec rosrun fast_lio build_map_scdb "${MAP_VALUE}" "${OUT_VALUE}" "$@"
