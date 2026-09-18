#!/usr/bin/env bash
# Build helper for the SUPER-based ROS1 Noetic workspace.
#
# Works with stock catkin_make only -- no catkin_tools required, and ccache is
# optional. This is the build entry point on the desktop container and on the
# Jetson Orin NX flight computers.
#
# Usage:
#   ./build.sh                  # full workspace build
#   ./build.sh lite             # build ONLY localization + control + message
#                               # deps (planning/simulation skipped). This sets
#                               # CATKIN_WHITELIST_PACKAGES; use `./build.sh`
#                               # or `./build.sh full` to clear it again.
#   ./build.sh --pkg fast_lio ekf_quat livox_ros_driver2 uav_utils px4ctrl
#                               # (re)build only the listed packages
#   ./build.sh --pkg mission_planner
#                               # dev loop: rebuild only the package you changed
#
# Stable packages (rarely change, compile once):
#   src/localization  (fast_lio, ekf_quat)
#   src/drivers       (livox_ros_driver2)
#   src/control       (px4ctrl, uav_utils, traj_server)
# Dev packages (the ones you iterate on):
#   src/super_planner, src/mission_planner, src/rog_map, src/mars_uav_sim
#
# lite mode builds: quadrotor_msgs livox_ros_driver2 fast_lio ekf_quat
#                   uav_utils px4ctrl traj_server
# Note: stale devel/ artifacts of packages built before a lite run are not
# removed; they are simply no longer rebuilt while the whitelist is active.
#
# Tips:
#   - First full build on an 8-core Orin NX takes a few minutes. Afterwards use
#     `./build.sh --pkg <pkg>` so stable packages are never recompiled.
#   - If you change a header inside a package others depend on, list that
#     package too, e.g. `./build.sh --pkg super_planner mission_planner`.
#   - Avoid editing .msg files in dev loops: message changes rebuild every
#     dependent package.
#   - RAM limits Jetson builds: 8 GB board -> BUILD_JOBS=4; 16 GB board -> the
#     default is fine. Add a swapfile if the build OOMs.
#   - Multi-drone deployment: compile once on one NX, then rsync build/ and
#     devel/ to the other drones. Each drone only recompiles what is newer.
#
# Env overrides:
#   BUILD_JOBS=N       force parallelism (default: 3/4 of nproc)
#   DISABLE_CCACHE=1   do not use ccache even if it is installed
set -eo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Source ROS first: its profile scripts reference unset vars (e.g.
# ROS_MASTER_URI) that would trip `set -u`.
source /opt/ros/noetic/setup.bash
set -u

# Parallelism: default to 3/4 of the available cores, override with BUILD_JOBS.
CORES="$(nproc)"
if [[ -n "${BUILD_JOBS:-}" ]]; then
    JOBS="${BUILD_JOBS}"
else
    JOBS=$(( CORES * 3 / 4 ))
    [[ "$JOBS" -lt 1 ]] && JOBS=1
fi

LITE_PACKAGES="quadrotor_msgs;livox_ros_driver2;fast_lio;ekf_quat;uav_utils;px4ctrl;traj_server"

MODE="full"
if [[ "${1:-}" == "lite" || "${1:-}" == "full" ]]; then
    MODE="$1"
    shift
fi

CMAKE_ARGS=()
if [[ "$MODE" == "lite" ]]; then
    echo "[build] LITE mode: only ${LITE_PACKAGES//;/ }"
    CMAKE_ARGS+=("-DCATKIN_WHITELIST_PACKAGES=$LITE_PACKAGES")
else
    echo "[build] FULL mode (clear package whitelist)"
    CMAKE_ARGS+=("-DCATKIN_WHITELIST_PACKAGES=")
fi

if command -v ccache >/dev/null 2>&1 && [[ "${DISABLE_CCACHE:-0}" != "1" ]]; then
    CMAKE_ARGS+=("-DCMAKE_CXX_COMPILER_LAUNCHER=ccache")
    echo "[build] ccache enabled (DISABLE_CCACHE=1 to disable)"
fi

# livox_ros_driver2: prefer the system-installed Livox-SDK2 (/usr/local) so the
# SDK is compiled once at provisioning time instead of on every driver rebuild
# (the SDK is ~100+ sources; the driver is a thin wrapper around it).
# Set LIVOX_USE_SYSTEM_SDK=OFF to force the vendored SDK on a machine whose
# /usr/local SDK is an incompatible version (no code change needed).
if [[ "${LIVOX_USE_SYSTEM_SDK:-auto}" == "OFF" || "${LIVOX_USE_SYSTEM_SDK:-auto}" == "0" ]]; then
    CMAKE_ARGS+=("-DLIVOX_USE_SYSTEM_SDK=OFF")
    echo "[build] livox_ros_driver2: vendored SDK (forced by LIVOX_USE_SYSTEM_SDK=OFF)"
elif [[ -f /usr/local/include/livox_lidar_api.h && -f /usr/local/lib/liblivox_lidar_sdk_static.a ]]; then
    CMAKE_ARGS+=("-DLIVOX_USE_SYSTEM_SDK=ON")
    echo "[build] livox_ros_driver2: system Livox-SDK2 (/usr/local)"
else
    CMAKE_ARGS+=("-DLIVOX_USE_SYSTEM_SDK=OFF")
    echo "[build] livox_ros_driver2: vendored SDK (system SDK not found in /usr/local)"
fi

echo "[build] jobs=${JOBS}  args=$*"
exec catkin_make "${CMAKE_ARGS[@]}" -j"$JOBS" -l"$JOBS" "$@"
