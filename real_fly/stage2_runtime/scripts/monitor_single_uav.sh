#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
dls_ws="${DLS_WS:-/home/nv/dls_ws}"

unset _CATKIN_SETUP_DIR || true
set +u
source /opt/ros/noetic/setup.bash
source "$dls_ws/devel/setup.bash"
set -u

exec python3 "$script_dir/monitor_single_uav.py" "$@"
