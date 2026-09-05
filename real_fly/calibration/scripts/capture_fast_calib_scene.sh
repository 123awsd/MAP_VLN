#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: capture_fast_calib_scene.sh --set-id ID --scene front|right|left
       [--duration-sec N] [--confirm-static-calibration]

Starts only an isolated ROS master, D435 color stream, and MID-360S. It checks
all four target ArUco markers, captures one image, and records /livox/lidar.
No MAVROS, flight controller, mapping, planner, or control node is started.
EOF
}

set_id=""
scene=""
duration=15
confirmed=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --set-id) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; set_id="$2"; shift 2 ;;
    --scene) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; scene="${2,,}"; shift 2 ;;
    --duration-sec) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; duration="$2"; shift 2 ;;
    --confirm-static-calibration) confirmed=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ "$set_id" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] || { echo "Invalid set ID." >&2; exit 2; }
[[ "$scene" == front || "$scene" == right || "$scene" == left ]] || {
  echo "Scene must be front, right, or left." >&2
  exit 2
}
[[ "$duration" =~ ^[1-9][0-9]*$ && "$duration" -le 60 ]] || {
  echo "Duration must be an integer from 1 to 60 seconds." >&2
  exit 2
}
[[ "$confirmed" -eq 1 ]] || {
  echo "Add --confirm-static-calibration after the aircraft is disarmed and the rig/board are secured." >&2
  exit 2
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CALIB_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
REAL_FLY_ROOT="$(cd "$CALIB_ROOT/.." && pwd)"
STAGE1_ROOT="$REAL_FLY_ROOT/stage1_exploration"
LIVOX_CONFIG="$STAGE1_ROOT/data/mid360s_static_20260904_145259/MID360s_config.json"
ROS_MASTER_PORT=11315
FINAL_DIR="$CALIB_ROOT/data/$set_id/$scene"
RUN_STAMP="$(date +%Y%m%d_%H%M%S)"
WORK_DIR="$CALIB_ROOT/runtime/capture/${set_id}_${scene}_${RUN_STAMP}"

[[ ! -e "$FINAL_DIR" ]] || { echo "Refusing to overwrite scene: $FINAL_DIR" >&2; exit 2; }
[[ -r "$LIVOX_CONFIG" ]] || { echo "Missing Livox config: $LIVOX_CONFIG" >&2; exit 1; }
mkdir -p "$WORK_DIR"

unset _CATKIN_SETUP_DIR || true
source /opt/ros/noetic/setup.bash
source "$STAGE1_ROOT/scripts/env.sh"
export ROS_MASTER_URI="http://127.0.0.1:${ROS_MASTER_PORT}"
export ROS_IP=127.0.0.1
export ROS_HOSTNAME=127.0.0.1

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
}
on_signal() { cleanup; exit 130; }
trap cleanup EXIT
trap on_signal INT TERM

start_owned() {
  local log_name="$1"
  shift
  "$@" >"$WORK_DIR/$log_name" 2>&1 &
  owned_pids+=("$!")
}
wait_topic() {
  local topic="$1" label="$2" deadline=$((SECONDS + 30))
  until timeout 3 rostopic echo -n 1 "$topic" >/dev/null 2>&1; do
    (( SECONDS < deadline )) || {
      echo "Timed out waiting for $label ($topic). Logs: $WORK_DIR" >&2
      return 1
    }
    sleep 1
  done
  echo "OK: $label ($topic)"
}

if rosnode list >/dev/null 2>&1; then
  echo "Port $ROS_MASTER_PORT already has a ROS master; stop that isolated session first." >&2
  exit 1
fi
start_owned roscore.log roscore -p "$ROS_MASTER_PORT"
for _ in {1..15}; do
  rosnode list >/dev/null 2>&1 && break
  sleep 1
done
rosnode list >/dev/null 2>&1 || { echo "Failed to start isolated ROS master." >&2; exit 1; }

start_owned d435.log roslaunch "$STAGE1_ROOT/launch/realsense_d435i.launch" \
  camera_name:=camera enable_color:=true enable_depth:=false \
  enable_accel:=false enable_gyro:=false enable_sync:=false \
  align_depth:=false publish_tf:=false

start_owned livox.log roslaunch "$STAGE1_ROOT/launch/livox_mid360.launch" \
  model:=mid360s config:="$LIVOX_CONFIG" publish_freq:=10.0

wait_topic /camera/color/image_raw "D435 color"
wait_topic /camera/color/camera_info "D435 color CameraInfo"
wait_topic /livox/lidar "MID-360S point cloud"

echo
echo "Scene: $scene"
echo "Keep the sensor rig and 1400x1000 mm board completely stationary."
echo "The complete board and all four ArUco markers must be visible in D435."
read -r -p "Press ENTER when the board is ready... "

"$SCRIPT_DIR/capture_camera_frame.py" \
  --image "$WORK_DIR/image.png" \
  --camera-info "$WORK_DIR/camera_info.json"

echo "Recording /livox/lidar for ${duration}s; do not move anything..."
rosbag record --lz4 -O "$WORK_DIR/lidar.bag" /livox/lidar \
  >"$WORK_DIR/rosbag.log" 2>&1 &
bag_pid=$!
owned_pids+=("$bag_pid")
sleep 1
kill -0 "$bag_pid" 2>/dev/null || { echo "rosbag recorder failed." >&2; exit 1; }
sleep "$duration"
kill -INT "$bag_pid" 2>/dev/null || true
wait "$bag_pid"

python3 - "$WORK_DIR/lidar.bag" <<'PY'
import rosbag
import sys

path = sys.argv[1]
with rosbag.Bag(path) as bag:
    info = bag.get_type_and_topic_info().topics.get('/livox/lidar')
    if info is None:
        raise SystemExit('missing /livox/lidar')
    if info.msg_type != 'livox_ros_driver2/CustomMsg':
        raise SystemExit('unexpected LiDAR type: ' + info.msg_type)
    if info.message_count < 50:
        raise SystemExit('too few LiDAR messages: {}'.format(info.message_count))
    print('LiDAR capture: {} messages, type {}'.format(info.message_count, info.msg_type))
PY

cleanup
trap - EXIT INT TERM
mkdir -p "$(dirname "$FINAL_DIR")"
mv "$WORK_DIR" "$FINAL_DIR"
echo "Scene capture complete: $FINAL_DIR"
