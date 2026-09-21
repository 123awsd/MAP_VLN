#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: start_localization_only.sh [--no-monitor]

Starts only the local ROS master when needed, MAVROS telemetry, MID-360,
lightweight FAST-LIO odometry, and a read-only health display. It does not
start a camera, RViz, a controller, a setpoint publisher, or map/point-cloud
publication. Start it only while the aircraft is disarmed.
EOF
}

with_monitor=1
while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-monitor) with_monitor=0; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUNTIME_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
REAL_FLY_ROOT="$(cd "$RUNTIME_ROOT/.." && pwd)"
STAGE1_ROOT="$REAL_FLY_ROOT/stage1_exploration"
DLS_WS=/home/nv/dls_ws
STAGE1_WS="$STAGE1_ROOT/ros_ws"
ROS_MASTER_PORT=11311
RUN_STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$RUNTIME_ROOT/runtime/localization/$RUN_STAMP"
mkdir -p "$LOG_DIR"

unset _CATKIN_SETUP_DIR || true
source /opt/ros/noetic/setup.bash

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
[[ -x "$STAGE1_WS/devel/lib/stage1_fast_lio/fastlio_mapping" ]] || {
  echo "FAST-LIO binary is missing; run stage1_exploration/scripts/build_stage1_fast_lio.sh." >&2
  exit 1
}
[[ -x "$DLS_WS/devel/lib/livox_ros_driver2/livox_ros_driver2_node" ]] || {
  echo "Livox driver binary is missing." >&2
  exit 1
}

owned_pids=()
cleanup() {
  local pid idx
  trap - EXIT INT TERM
  set +e
  for ((idx=${#owned_pids[@]}-1; idx>=0; idx--)); do
    pid="${owned_pids[$idx]}"
    kill -INT "$pid" 2>/dev/null || true
  done
  for ((idx=${#owned_pids[@]}-1; idx>=0; idx--)); do
    wait "${owned_pids[$idx]}" 2>/dev/null || true
  done
  echo "Stopped only this launch's child processes. Logs: $LOG_DIR"
}
on_signal() {
  cleanup
  exit 130
}
trap cleanup EXIT
trap on_signal INT TERM

start_owned() {
  local log_file="$1"
  shift
  "$@" >"$LOG_DIR/$log_file" 2>&1 &
  owned_pids+=("$!")
}
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
  local topic="$1" label="$2" deadline=$((SECONDS + 25))
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

echo "Safety mode: localization telemetry only; no camera, mapping output, or control."
echo "Logs: $LOG_DIR"

if ! rosnode list >/dev/null 2>&1; then
  echo "Starting roscore on $ROS_MASTER_URI"
  start_owned roscore.log roscore -p "$ROS_MASTER_PORT"
  wait_for_master
else
  echo "Reusing ROS master: $ROS_MASTER_URI"
fi

if node_exists /mavros; then
  echo "Reusing /mavros"
else
  echo "Starting MAVROS telemetry (/dev/ttyTHS0:921600)"
  start_owned mavros.log roslaunch mavros px4.launch
fi
wait_for_topic /mavros/imu/data "flight-controller IMU"

state="$(timeout 8 rostopic echo -n 1 /mavros/state 2>/dev/null || true)"
grep -q '^connected: True' <<<"$state" || {
  echo "MAVROS is running but the flight controller is not connected." >&2
  exit 1
}
if grep -q '^armed: True' <<<"$state"; then
  echo "Refusing startup because MAVROS reports armed: True." >&2
  exit 1
fi
echo "OK: flight controller connected and disarmed"

echo "Requesting FCU HIGHRES_IMU telemetry at 200 Hz (telemetry setup only)"
if ! timeout 10 rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0 \
    >"$LOG_DIR/imu_rate_setup.log" 2>&1; then
  echo "Failed to request 200 Hz HIGHRES_IMU; inspect $LOG_DIR/imu_rate_setup.log" >&2
  exit 1
fi

if node_exists /livox_lidar_publisher2; then
  echo "Reusing /livox_lidar_publisher2"
else
  echo "Starting $LIVOX_LABEL at $LIVOX_IP"
  start_owned livox.log roslaunch "$DLS_WS/src/localization/FAST_LIO/launch/lidar.launch"
fi
wait_for_topic /livox/lidar "Livox point cloud"

if node_exists /laserMapping; then
  echo "Refusing to reuse /laserMapping because its active profile is unknown." >&2
  echo "Stop the existing FAST-LIO process and run this command again." >&2
  exit 1
fi
echo "Starting lightweight FAST-LIO localization"
start_owned fastlio.log roslaunch stage1_fast_lio localization_only.launch
wait_for_topic /Odometry "FAST-LIO odometry"

echo
echo "Localization is ready: /Odometry and TF world -> body"
echo "No point cloud, global map, path, camera, RViz, or controller was started."

if [[ "$with_monitor" -eq 1 ]]; then
  "$SCRIPT_DIR/monitor_localization.py"
else
  echo "Running without terminal monitor. Press Ctrl-C to stop."
  while :; do sleep 2; done
fi
