#!/usr/bin/env bash
set -euo pipefail

usage() { echo "Usage: $0 --set-id ID"; }
set_id=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --set-id) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; set_id="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done
[[ "$set_id" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] || { echo "Invalid set ID." >&2; exit 2; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CALIB_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
STAGE1_ROOT="$(cd "$CALIB_ROOT/../stage1_exploration" && pwd)"
ROS_WS="$CALIB_ROOT/ros_ws"
CONFIG="$CALIB_ROOT/config/d435_mid360_1400x1000.yaml"
RESULTS_ROOT="$CALIB_ROOT/data/$set_id/results"
OUTPUT_DIR="$RESULTS_ROOT/multi"
ROS_MASTER_PORT=11316

[[ ! -e "$OUTPUT_DIR" ]] || { echo "Refusing to overwrite result: $OUTPUT_DIR" >&2; exit 2; }
for scene in front right left; do
  [[ -s "$RESULTS_ROOT/$scene/circle_center_record.txt" ]] || {
    echo "Missing successful single-scene result: $scene" >&2
    exit 1
  }
done
mkdir -p "$OUTPUT_DIR" "$CALIB_ROOT/runtime/ros_home"
for scene in front right left; do
  /bin/cat "$RESULTS_ROOT/$scene/circle_center_record.txt"
done >"$OUTPUT_DIR/circle_center_record.txt"

unset _CATKIN_SETUP_DIR || true
source /opt/ros/noetic/setup.bash
source "$STAGE1_ROOT/scripts/env.sh"
source "$ROS_WS/devel/setup.bash"
export ROS_MASTER_URI="http://127.0.0.1:${ROS_MASTER_PORT}"
export ROS_IP=127.0.0.1
export ROS_HOSTNAME=127.0.0.1
export ROS_HOME="$CALIB_ROOT/runtime/ros_home"

roscore -p "$ROS_MASTER_PORT" >"$OUTPUT_DIR/roscore.log" 2>&1 &
master_pid=$!
cleanup() { set +e; kill -INT "$master_pid" 2>/dev/null || true; wait "$master_pid" 2>/dev/null || true; }
on_signal() { exit 130; }
trap cleanup EXIT
trap on_signal INT TERM
for _ in {1..15}; do rosparam list >/dev/null 2>&1 && break; sleep 1; done
rosparam list >/dev/null 2>&1 || { echo "Failed to start ROS master." >&2; exit 1; }

rosparam load "$CONFIG"
rosparam set /output_path "$OUTPUT_DIR/"
rosrun fast_calib multi_fast_calib >"$OUTPUT_DIR/multi_fast_calib.log" 2>&1
[[ -s "$OUTPUT_DIR/multi_calib_result.txt" ]] || {
  echo "Multi-scene calibration failed. Log: $OUTPUT_DIR/multi_fast_calib.log" >&2
  exit 1
}
echo "Final T_cam_lidar: $OUTPUT_DIR/multi_calib_result.txt"
