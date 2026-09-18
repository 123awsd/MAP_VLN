#!/usr/bin/env bash
# Start only the Livox LiDAR driver (fast_lio/lidar.launch).
#
# This script always uses THIS workspace (dls_ws) and loads the drone/lidar
# environment from /etc/uav/uav.env, so it works regardless of what shell /
# workspace the caller has sourced. Extra roslaunch args are forwarded, e.g.:
#   ./lidar.sh
#   ./lidar.sh publish_freq:=10.0
set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Drone/lidar env (LIVOX_LIDAR_TYPE, LIVOX_LIDAR_IP, ...) from /etc/uav/uav.env.
# Exported vars already present in the environment win over the file.
source "${WS_DIR}/scripts/load_uav_env.sh"

# ROS setup scripts reference vars (e.g. ROS_DISTRO) that may not be defined in
# a fresh shell, so source them before enabling strict "set -u".
source /opt/ros/noetic/setup.bash
source "${WS_DIR}/devel/setup.bash"

set -euo pipefail

# Sanity check: make sure we are running THIS workspace's packages.
if ! rospack find fast_lio >/dev/null 2>&1; then
    echo "[lidar.sh] ERROR: cannot find fast_lio package. Is dls_ws built?" >&2
    exit 1
fi

echo "[lidar.sh] fast_lio: $(rospack find fast_lio)"
echo "[lidar.sh] livox_ros_driver2: $(rospack find livox_ros_driver2)"
echo "[lidar.sh] LIVOX_LIDAR_TYPE=${LIVOX_LIDAR_TYPE:-<unset>} LIVOX_LIDAR_IP=${LIVOX_LIDAR_IP:-<unset>}"

exec roslaunch fast_lio lidar.launch "$@"
