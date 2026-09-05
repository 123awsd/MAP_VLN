#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: start_handheld_mapping_rviz.sh [--record RUN_ID] [--no-camera] [--no-rviz]

Starts the existing ROS master (or a private one when none exists), MAVROS
telemetry, the aircraft MID-360S, the borrowed FAST-LIO mapping profile, the
D435 RGB-D stream, and the checked-in RViz view. It never arms the aircraft or
publishes motion commands.

Close RViz or press Ctrl-C to stop only the processes started by this script.
With --record, recording begins only after all requested streams and FAST-LIO
are ready, and is stored below data/RUN_ID/raw/.
EOF
}

with_camera=1
with_rviz=1
record_run_id=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --record) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; record_run_id="$2"; shift 2 ;;
    --no-camera) with_camera=0; shift ;;
    --no-rviz) with_rviz=0; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ -n "$record_run_id" && ! "$record_run_id" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]]; then
  echo "Invalid recording run ID: $record_run_id" >&2
  exit 2
fi
if [[ -n "$record_run_id" && "$with_camera" -eq 0 ]]; then
  echo "--record requires the D435; do not combine it with --no-camera." >&2
  exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STAGE1_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
DLS_WS=/home/nv/dls_ws
STAGE1_WS="$STAGE1_ROOT/ros_ws"
ROS_MASTER_PORT=11311
LIVOX_CONFIG="$STAGE1_ROOT/data/mid360s_static_20260904_145259/MID360s_config.json"
LIVOX_SERIAL=ARMCP720033122
RVIZ_CONFIG="$STAGE1_ROOT/rviz/stage1_handheld_mapping.rviz"
RUN_STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$STAGE1_ROOT/runtime/live_mapping/$RUN_STAMP"
mkdir -p "$LOG_DIR"

# A stale exported catkin helper variable can make setup.bash resolve the wrong
# setup.sh when this script is launched from an IDE terminal.
unset _CATKIN_SETUP_DIR || true
source /opt/ros/noetic/setup.bash

export ROS_MASTER_URI="http://127.0.0.1:${ROS_MASTER_PORT}"
export ROS_IP=127.0.0.1
export ROS_HOSTNAME=127.0.0.1
export ROS_PACKAGE_PATH="$STAGE1_WS/src:$DLS_WS/src:/opt/ros/noetic/share"
export CMAKE_PREFIX_PATH="$STAGE1_WS/devel:$DLS_WS/devel:/opt/ros/noetic"
export PATH="$STAGE1_WS/devel/lib/stage1_fast_lio:$DLS_WS/devel/lib/livox_ros_driver2:/opt/ros/noetic/bin:$PATH"
export LD_LIBRARY_PATH="$STAGE1_WS/devel/lib:$DLS_WS/devel/lib:/opt/ros/noetic/lib:/opt/ros/noetic/lib/aarch64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PYTHONPATH="$STAGE1_WS/devel/lib/python3/dist-packages:$DLS_WS/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
export LIVOX_LIDAR_TYPE=mid360s
export LIVOX_LIDAR_CONFIG="$LIVOX_CONFIG"

[[ -r "$LIVOX_CONFIG" ]] || { echo "Missing Livox config: $LIVOX_CONFIG" >&2; exit 1; }
[[ -r "$RVIZ_CONFIG" ]] || { echo "Missing RViz config: $RVIZ_CONFIG" >&2; exit 1; }
[[ -x "$STAGE1_WS/devel/lib/stage1_fast_lio/fastlio_mapping" ]] || { echo "Stage-1 FAST-LIO binary is missing; run scripts/build_stage1_fast_lio.sh." >&2; exit 1; }
[[ -x "$DLS_WS/devel/lib/livox_ros_driver2/livox_ros_driver2_node" ]] || { echo "Livox driver binary is missing." >&2; exit 1; }

owned_pids=()
cleanup() {
  local pid
  trap - EXIT INT TERM
  set +e
  for ((idx=${#owned_pids[@]}-1; idx>=0; idx--)); do
    pid="${owned_pids[$idx]}"
    kill -INT "$pid" 2>/dev/null || true
  done
  for ((idx=${#owned_pids[@]}-1; idx>=0; idx--)); do
    pid="${owned_pids[$idx]}"
    wait "$pid" 2>/dev/null || true
  done
  echo "Stopped only this launch's child processes. Logs: $LOG_DIR"
}
on_signal() {
  cleanup
  exit 130
}
trap cleanup EXIT
trap on_signal INT TERM

node_exists() {
  rosnode list 2>/dev/null | grep -Fxq "$1"
}

wait_for_master() {
  local deadline=$((SECONDS + 15))
  until rosnode list >/dev/null 2>&1; do
    (( SECONDS < deadline )) || { echo "ROS master did not become ready." >&2; return 1; }
    sleep 1
  done
}

wait_for_topic() {
  local topic="$1"
  local label="$2"
  local deadline=$((SECONDS + 45))
  until rostopic info "$topic" 2>/dev/null | grep -q '^Publishers:'; do
    (( SECONDS < deadline )) || { echo "Timed out waiting for $label ($topic)." >&2; return 1; }
    sleep 1
  done
  timeout 8 rostopic echo -n 1 "$topic" >/dev/null 2>&1 || {
    echo "$label exists but no message arrived: $topic" >&2
    return 1
  }
  echo "OK: $label ($topic)"
}

stop_owned_pid() {
  local pid="$1"
  kill -INT "$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
}

start_launch() {
  local log_file="$1"
  shift
  "$@" >"$LOG_DIR/$log_file" 2>&1 &
  owned_pids+=("$!")
}

echo "Safety mode: telemetry and mapping only; no arming or motion command."
echo "Logs: $LOG_DIR"

if ! rosnode list >/dev/null 2>&1; then
  echo "Starting roscore on ${ROS_MASTER_URI}"
  start_launch roscore.log roscore -p "$ROS_MASTER_PORT"
  wait_for_master
else
  echo "Reusing ROS master: $ROS_MASTER_URI"
fi

if node_exists /mavros; then
  echo "Reusing /mavros"
else
  echo "Starting MAVROS telemetry (/dev/ttyTHS0:921600)"
  start_launch mavros.log roslaunch mavros px4.launch
fi
wait_for_topic /mavros/imu/data "flight-controller IMU"

state="$(timeout 8 rostopic echo -n 1 /mavros/state 2>/dev/null || true)"
if grep -q '^armed: True' <<<"$state"; then
  echo "Refusing mapping startup because MAVROS reports armed: True." >&2
  exit 1
fi
grep -q '^connected: True' <<<"$state" || { echo "MAVROS is running but the flight controller is not connected." >&2; exit 1; }
echo "OK: flight controller connected and disarmed"

echo "Requesting FCU HIGHRES_IMU telemetry at 200 Hz (no control command)"
if ! timeout 10 rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0 \
    >"$LOG_DIR/imu_rate_setup.log" 2>&1; then
  echo "Failed to request 200 Hz HIGHRES_IMU; inspect $LOG_DIR/imu_rate_setup.log" >&2
  exit 1
fi

if node_exists /livox_lidar_publisher2; then
  echo "Reusing /livox_lidar_publisher2"
  wait_for_topic /livox/lidar "Livox point cloud"
else
  livox_ready=0
  for attempt in 1 2; do
    echo "Starting MID-360S at 192.168.1.122 (attempt $attempt/2)"
    livox_log="livox.log"
    [[ "$attempt" -eq 1 ]] || livox_log="livox_retry.log"
    start_launch "$livox_log" roslaunch "$DLS_WS/src/localization/FAST_LIO/launch/lidar.launch" \
      "bd_list:=$LIVOX_SERIAL"
    livox_pid="${owned_pids[-1]}"
    if wait_for_topic /livox/lidar "Livox point cloud"; then
      livox_ready=1
      break
    fi
    stop_owned_pid "$livox_pid"
    [[ "$attempt" -eq 2 ]] || sleep 5
  done
  [[ "$livox_ready" -eq 1 ]] || {
    echo "MID-360S did not produce point cloud after two attempts." >&2
    exit 1
  }
fi

if [[ "$with_camera" -eq 1 ]]; then
  if node_exists /camera/realsense2_camera_manager || node_exists /camera/realsense2_camera; then
    echo "Reusing D435 ROS node"
  else
    rospack find realsense2_camera >/dev/null 2>&1 || {
      echo "realsense2_camera is unavailable; use --no-camera for LiDAR-only mapping." >&2
      exit 1
    }
    echo "Starting D435 RGB-D"
    start_launch d435.log roslaunch "$STAGE1_ROOT/launch/realsense_d435i.launch" \
      camera_name:=camera enable_color:=true enable_depth:=true \
      enable_accel:=false enable_gyro:=false enable_sync:=false align_depth:=true \
      depth_width:=640 depth_height:=480 depth_fps:=15 \
      color_width:=640 color_height:=480 color_fps:=15
  fi
  wait_for_topic /camera/color/image_raw "D435 RGB"
  wait_for_topic /camera/depth/image_rect_raw "D435 raw depth"
  wait_for_topic /camera/aligned_depth_to_color/image_raw "D435 aligned depth"
fi

if node_exists /laserMapping; then
  echo "Reusing /laserMapping"
else
  echo "Starting tuned Stage-1 FAST-LIO with FCU IMU and live map"
  start_launch fastlio.log roslaunch stage1_fast_lio live_mapping.launch \
    "map_name:=handheld_${RUN_STAMP}.pcd"
fi
wait_for_topic /cloud_registered "FAST-LIO registered cloud"

if [[ "$with_camera" -eq 1 ]]; then
  echo "Starting calibrated RGB coverage overlay"
  start_launch rgb_coverage.log "$SCRIPT_DIR/rgb_coverage_visualizer.py"
  wait_for_topic /rgb_coverage/current_frustum "RGB current-view overlay"
fi

if [[ -n "$record_run_id" ]]; then
  RECORD_DIR="$STAGE1_ROOT/data/$record_run_id"
  [[ ! -e "$RECORD_DIR" ]] || {
    echo "Refusing to overwrite existing run: $RECORD_DIR" >&2
    exit 2
  }
  mkdir -p "$RECORD_DIR/raw"
  cat >"$RECORD_DIR/manifest.txt" <<EOF
run_id=$record_run_id
started_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)
ros_master_uri=$ROS_MASTER_URI
lidar=/livox/lidar
mapping_imu=/mavros/imu/data
rgb=/camera/color/image_raw
depth=/camera/aligned_depth_to_color/image_raw
depth_raw=/camera/depth/image_rect_raw
odometry=/Odometry
registered_cloud=/cloud_registered
EOF
  echo "Recording synchronized raw sensors, RGB-D, pose, TF and map outputs"
  start_launch rosbag.log rosbag record --lz4 \
    -O "$RECORD_DIR/raw/sensors.bag" \
    /livox/lidar /livox/imu /mavros/imu/data \
    /camera/color/image_raw /camera/depth/image_rect_raw \
    /camera/aligned_depth_to_color/image_raw \
    /camera/color/camera_info /camera/depth/camera_info \
    /camera/aligned_depth_to_color/camera_info \
    /camera/extrinsics/depth_to_color \
    /tf /tf_static /Odometry /cloud_registered \
    /rgb_coverage/frustums /rgb_coverage/camera_path \
    /rgb_coverage/current_frustum /rgb_coverage/count
  sleep 2
  kill -0 "${owned_pids[-1]}" 2>/dev/null || {
    echo "rosbag recorder exited; inspect $LOG_DIR/rosbag.log" >&2
    exit 1
  }
  echo "RECORDING: $RECORD_DIR/raw/sensors.bag"
fi

echo
echo "Mapping is ready. Keep the rig still for 5 seconds, then move slowly."
echo "RViz: Fixed Frame=world, PointCloud2=/cloud_registered"

if [[ "$with_rviz" -eq 1 ]]; then
  [[ -n "${DISPLAY:-}" ]] || { echo "DISPLAY is unset. Run this command inside a NoMachine terminal." >&2; exit 1; }
  rviz -d "$RVIZ_CONFIG"
else
  echo "Running without RViz. Press Ctrl-C to stop."
  while :; do sleep 2; done
fi
