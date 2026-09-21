#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: start_handheld_mapping_rviz.sh [--record RUN_ID] [--no-camera] [--no-rviz]

Starts the existing ROS master (or a private one when none exists), MAVROS
telemetry, the aircraft MID-360, the borrowed FAST-LIO mapping profile, the
senior-provided ekf_quat fusion node, the D435 RGB-D stream, and the checked-in
RViz view. It never arms the aircraft or publishes motion commands.

Close RViz or press Ctrl-C to stop only the processes started by this script.
With --record, raw recording starts before a fresh FAST-LIO initialization so
offline replay contains the complete stationary pre-roll. Output is stored
below data/RUN_ID/raw/.
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
RVIZ_CONFIG="$STAGE1_ROOT/rviz/stage1_handheld_mapping.rviz"
REALSENSE_PROFILE="$STAGE1_ROOT/config/realsense_d435_recording.conf"
RUN_STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$STAGE1_ROOT/runtime/live_mapping/$RUN_STAMP"
mkdir -p "$LOG_DIR"

# A stale exported catkin helper variable can make setup.bash resolve the wrong
# setup.sh when this script is launched from an IDE terminal.
unset _CATKIN_SETUP_DIR || true
# ROS profile hooks inspect variables that are intentionally absent in a fresh
# non-interactive SSH shell, so do not apply nounset while sourcing them.
set +u
source /opt/ros/noetic/setup.bash
set -u

# Reuse the same per-aircraft Livox settings as the senior localization stack.
# This avoids overriding the new MID-360 with the previous MID-360S model and
# broadcast code.
if [[ -r "$DLS_WS/scripts/load_uav_env.sh" ]]; then
  # shellcheck disable=SC1090
  source "$DLS_WS/scripts/load_uav_env.sh"
fi
LIVOX_MODEL="${LIVOX_LIDAR_TYPE:-mid360}"
LIVOX_IP="${LIVOX_LIDAR_IP:-192.168.1.139}"
LIVOX_CONFIG="${LIVOX_LIDAR_CONFIG:-$DLS_WS/src/drivers/livox_ros_driver2/config/MID360_config.json}"
if [[ "$LIVOX_MODEL" == "mid360" ]]; then
  LIVOX_LABEL="MID-360"
else
  LIVOX_LABEL="MID-360S"
fi

export ROS_MASTER_URI="http://127.0.0.1:${ROS_MASTER_PORT}"
export ROS_IP=127.0.0.1
export ROS_HOSTNAME=127.0.0.1
export ROS_PACKAGE_PATH="$STAGE1_WS/src:$DLS_WS/src:/opt/ros/noetic/share"
export CMAKE_PREFIX_PATH="$STAGE1_WS/devel:$DLS_WS/devel:/opt/ros/noetic"
export PATH="$STAGE1_WS/devel/lib/stage1_fast_lio:$DLS_WS/devel/lib/livox_ros_driver2:/opt/ros/noetic/bin:$PATH"
export LD_LIBRARY_PATH="$STAGE1_WS/devel/lib:$DLS_WS/devel/lib:/opt/ros/noetic/lib:/opt/ros/noetic/lib/aarch64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PYTHONPATH="$STAGE1_WS/devel/lib/python3/dist-packages:$DLS_WS/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
export LIVOX_LIDAR_TYPE="$LIVOX_MODEL"
export LIVOX_LIDAR_IP="$LIVOX_IP"
export LIVOX_LIDAR_CONFIG="$LIVOX_CONFIG"

[[ -r "$LIVOX_CONFIG" ]] || { echo "Missing Livox config: $LIVOX_CONFIG" >&2; exit 1; }
[[ -r "$RVIZ_CONFIG" ]] || { echo "Missing RViz config: $RVIZ_CONFIG" >&2; exit 1; }
[[ -r "$REALSENSE_PROFILE" ]] || { echo "Missing RealSense profile: $REALSENSE_PROFILE" >&2; exit 1; }
# shellcheck disable=SC1090
source "$REALSENSE_PROFILE"
[[ -x "$STAGE1_WS/devel/lib/stage1_fast_lio/fastlio_mapping" ]] || { echo "Stage-1 FAST-LIO binary is missing; run scripts/build_stage1_fast_lio.sh." >&2; exit 1; }
[[ -x "$DLS_WS/devel/lib/livox_ros_driver2/livox_ros_driver2_node" ]] || { echo "Livox driver binary is missing." >&2; exit 1; }

owned_pids=()
record_pid=""
cleanup() {
  local pid priority_record_pid
  trap - EXIT INT TERM
  set +e
  # Stop rosbag first so derived topics are finalized while their publishers
  # are still alive. The same PID remains in owned_pids and is skipped below.
  priority_record_pid="$record_pid"
  if [[ -n "$priority_record_pid" ]]; then
    kill -INT "$priority_record_pid" 2>/dev/null || true
    wait "$priority_record_pid" 2>/dev/null || true
  fi
  for ((idx=${#owned_pids[@]}-1; idx>=0; idx--)); do
    pid="${owned_pids[$idx]}"
    [[ "$pid" == "$priority_record_pid" ]] && continue
    kill -INT "$pid" 2>/dev/null || true
  done
  for ((idx=${#owned_pids[@]}-1; idx>=0; idx--)); do
    pid="${owned_pids[$idx]}"
    [[ "$pid" == "$priority_record_pid" ]] && continue
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
    echo "Starting $LIVOX_LABEL at $LIVOX_IP (attempt $attempt/2)"
    livox_log="livox.log"
    [[ "$attempt" -eq 1 ]] || livox_log="livox_retry.log"
    start_launch "$livox_log" roslaunch "$DLS_WS/src/localization/FAST_LIO/launch/lidar.launch"
    livox_pid="${owned_pids[-1]}"
    if wait_for_topic /livox/lidar "Livox point cloud"; then
      livox_ready=1
      break
    fi
    stop_owned_pid "$livox_pid"
    [[ "$attempt" -eq 2 ]] || sleep 5
  done
  [[ "$livox_ready" -eq 1 ]] || {
    echo "$LIVOX_LABEL did not produce point cloud after two attempts." >&2
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
      color_width:="$REALSENSE_COLOR_WIDTH" color_height:="$REALSENSE_COLOR_HEIGHT" \
      color_fps:="$REALSENSE_COLOR_FPS" depth_width:="$REALSENSE_DEPTH_WIDTH" \
      depth_height:="$REALSENSE_DEPTH_HEIGHT" depth_fps:="$REALSENSE_DEPTH_FPS"
  fi
  wait_for_topic /camera/color/image_raw "D435 RGB"
  wait_for_topic /camera/depth/image_rect_raw "D435 raw depth"
  wait_for_topic /camera/aligned_depth_to_color/image_raw "D435 aligned depth"
  "$SCRIPT_DIR/configure_realsense_rgb.sh"
fi

if [[ -n "$record_run_id" ]]; then
  # A formal recording must have a reproducible cold estimator start. Reusing
  # either node would make the saved online state impossible to reconstruct
  # from the beginning of the Bag.
  if node_exists /laserMapping || node_exists /ekf_quat; then
    echo "Refusing formal recording while /laserMapping or /ekf_quat already exists." >&2
    echo "Stop the previous mapping launch, then run this command again." >&2
    exit 2
  fi

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
rgb_profile=$REALSENSE_PROFILE
rgb_auto_exposure=$REALSENSE_RGB_AUTO_EXPOSURE
rgb_exposure=$REALSENSE_RGB_EXPOSURE
odometry_raw=/Odometry
odometry_ekf=/ekf_quat/ekf_odom
registered_cloud=/cloud_registered
EOF
  echo "Starting raw recording before FAST-LIO initialization"
  start_launch rosbag.log rosbag record --lz4 \
    -O "$RECORD_DIR/raw/sensors.bag" \
    /livox/lidar /livox/imu /mavros/imu/data \
    /camera/color/image_raw /camera/depth/image_rect_raw \
    /camera/aligned_depth_to_color/image_raw \
    /camera/color/camera_info /camera/depth/camera_info \
    /camera/aligned_depth_to_color/camera_info \
    /camera/extrinsics/depth_to_color \
    /tf /tf_static /Odometry /ekf_quat/ekf_odom /cloud_registered \
    /rgb_coverage/frustums /rgb_coverage/camera_path \
    /rgb_coverage/current_frustum /rgb_coverage/count
  record_pid="${owned_pids[-1]}"
  sleep 2
  kill -0 "$record_pid" 2>/dev/null || {
    echo "rosbag recorder exited; inspect $LOG_DIR/rosbag.log" >&2
    exit 1
  }
  echo "RECORDING: $RECORD_DIR/raw/sensors.bag"
  echo "Keep the rig stationary: capturing 8 seconds of raw initialization data."
  sleep 8
fi

if node_exists /laserMapping; then
  echo "Refusing to reuse /laserMapping because its active self-filter profile is unknown." >&2
  echo "Stop the existing FAST-LIO process and run handheld mapping again." >&2
  exit 1
fi
echo "Starting tuned Stage-1 FAST-LIO with handheld-only rear self filter"
start_launch fastlio.log roslaunch stage1_fast_lio live_mapping.launch \
  "map_name:=handheld_${RUN_STAMP}.pcd"
wait_for_topic /cloud_registered "FAST-LIO registered cloud"

# Use the senior-provided EKF as the downstream pose consumed by later
# semantic/localization stages.  Keep raw /Odometry available for diagnostics.
EKF_LAUNCH="$DLS_WS/src/localization/ekf_quat_pose/launch/ekf_quat_lidar_mavros.launch"
[[ -r "$EKF_LAUNCH" ]] || {
  echo "Missing senior EKF launch file: $EKF_LAUNCH" >&2
  exit 1
}
if node_exists /ekf_quat; then
  echo "Reusing senior EKF /ekf_quat"
else
  echo "Starting senior EKF fusion (/ekf_quat/ekf_odom)"
  start_launch ekf_quat.log roslaunch "$EKF_LAUNCH"
fi
wait_for_topic /ekf_quat/ekf_odom "senior EKF fused odometry"

if [[ "$with_camera" -eq 1 ]]; then
  echo "Starting calibrated RGB coverage overlay"
  start_launch rgb_coverage.log "$SCRIPT_DIR/rgb_coverage_visualizer.py"
  wait_for_topic /rgb_coverage/current_frustum "RGB current-view overlay"
fi

echo
echo "Mapping is ready. Keep the rig still for 5 seconds, then move slowly."
echo "RViz: Fixed Frame=world, PointCloud2=/cloud_registered"
echo "Starting read-only FAST-LIO coordinate display (/Odometry, 1 Hz)"
python3 "$SCRIPT_DIR/monitor_handheld_pose.py" --topic /Odometry --rate-hz 1.0 &
owned_pids+=("$!")

if [[ "$with_rviz" -eq 1 ]]; then
  [[ -n "${DISPLAY:-}" ]] || { echo "DISPLAY is unset. Run this command inside a NoMachine terminal." >&2; exit 1; }
  rviz -d "$RVIZ_CONFIG"
else
  echo "Running without RViz. Press Ctrl-C to stop."
  while :; do sleep 2; done
fi
