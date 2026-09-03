#!/usr/bin/env bash
# Source this file in a child shell for the Stage-1 overlay. It deliberately
# does not set ROS_MASTER_URI, ROS_IP, or any persistent system setting.

if [[ -z "${BASH_VERSION:-}" ]]; then
  echo "This helper must be sourced by bash." >&2
  return 2 2>/dev/null || exit 2
fi

STAGE1_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DLS_WS="/home/nv/dls_ws"
STAGE1_WS="${STAGE1_ROOT}/ros_ws"

export STAGE1_ROOT
export STAGE1_WS
export DLS_WS
export PATH="/opt/ros/noetic/bin:${STAGE1_WS}/devel/bin:${DLS_WS}/devel/bin:${PATH}"
export LD_LIBRARY_PATH="${STAGE1_WS}/devel/lib:${DLS_WS}/devel/lib:/usr/local/lib:/opt/ros/noetic/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export CMAKE_PREFIX_PATH="${STAGE1_WS}/devel:${DLS_WS}/devel:/opt/ros/noetic${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
export ROS_PACKAGE_PATH="${STAGE1_WS}/src:${DLS_WS}/src:/opt/ros/noetic/share${ROS_PACKAGE_PATH:+:${ROS_PACKAGE_PATH}}"
export PYTHONPATH="${STAGE1_WS}/devel/lib/python3/dist-packages:${DLS_WS}/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages${PYTHONPATH:+:${PYTHONPATH}}"
export PKG_CONFIG_PATH="${STAGE1_WS}/devel/lib/pkgconfig:${DLS_WS}/devel/lib/pkgconfig:/opt/ros/noetic/lib/pkgconfig${PKG_CONFIG_PATH:+:${PKG_CONFIG_PATH}}"
export ROS_HOME="${STAGE1_ROOT}/runtime/ros_home"
export ROS_LOG_DIR="${STAGE1_ROOT}/runtime/ros_log"

# No default IP is exported. These are consumed only by an explicit sensor
# launch after the actual LAN1 and MID-360 addresses have been identified.
