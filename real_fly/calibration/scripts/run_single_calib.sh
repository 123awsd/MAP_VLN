#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: run_single_calib.sh --set-id ID --scene front|right|left
       [--x-min V --x-max V --y-min V --y-max V --z-min V --z-max V]

Runs FAST-Calib on one previously captured static scene. The default crop is
x=[1.5,3.5], y=[-1.2,1.2], z=[-1.2,1.2] in the LiDAR frame.
EOF
}

set_id=""
scene=""
x_min=1.5; x_max=3.5
y_min=-1.2; y_max=1.2
z_min=-1.2; z_max=1.2
while [[ $# -gt 0 ]]; do
  case "$1" in
    --set-id) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; set_id="$2"; shift 2 ;;
    --scene) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; scene="${2,,}"; shift 2 ;;
    --x-min) x_min="$2"; shift 2 ;; --x-max) x_max="$2"; shift 2 ;;
    --y-min) y_min="$2"; shift 2 ;; --y-max) y_max="$2"; shift 2 ;;
    --z-min) z_min="$2"; shift 2 ;; --z-max) z_max="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ "$set_id" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] || { echo "Invalid set ID." >&2; exit 2; }
[[ "$scene" == front || "$scene" == right || "$scene" == left ]] || { echo "Invalid scene." >&2; exit 2; }
number_re='^-?[0-9]+([.][0-9]+)?$'
for value in "$x_min" "$x_max" "$y_min" "$y_max" "$z_min" "$z_max"; do
  [[ "$value" =~ $number_re ]] || { echo "Invalid numeric filter value: $value" >&2; exit 2; }
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CALIB_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
STAGE1_ROOT="$(cd "$CALIB_ROOT/../stage1_exploration" && pwd)"
ROS_WS="$CALIB_ROOT/ros_ws"
CONFIG="$CALIB_ROOT/config/d435_mid360_1400x1000.yaml"
SCENE_DIR="$CALIB_ROOT/data/$set_id/$scene"
OUTPUT_DIR="$CALIB_ROOT/data/$set_id/results/$scene"
ROS_MASTER_PORT=11316

[[ -f "$SCENE_DIR/lidar.bag" && -f "$SCENE_DIR/image.png" ]] || {
  echo "Incomplete scene capture: $SCENE_DIR" >&2
  exit 1
}
[[ ! -e "$OUTPUT_DIR" ]] || { echo "Refusing to overwrite result: $OUTPUT_DIR" >&2; exit 2; }
[[ -x "$ROS_WS/devel/lib/fast_calib/fast_calib" ]] || {
  echo "FAST-Calib is not built; run setup_fast_calib.sh first." >&2
  exit 1
}
mkdir -p "$OUTPUT_DIR" "$CALIB_ROOT/runtime/ros_home"

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
calib_pid=""
cleanup() {
  set +e
  [[ -z "$calib_pid" ]] || kill -INT "$calib_pid" 2>/dev/null || true
  kill -INT "$master_pid" 2>/dev/null || true
  [[ -z "$calib_pid" ]] || wait "$calib_pid" 2>/dev/null || true
  wait "$master_pid" 2>/dev/null || true
}
on_signal() { exit 130; }
trap cleanup EXIT
trap on_signal INT TERM
for _ in {1..15}; do rosparam list >/dev/null 2>&1 && break; sleep 1; done
rosparam list >/dev/null 2>&1 || { echo "Failed to start ROS master." >&2; exit 1; }

rosparam load "$CONFIG"
rosparam set /bag_path "$SCENE_DIR/lidar.bag"
rosparam set /image_path "$SCENE_DIR/image.png"
rosparam set /output_path "$OUTPUT_DIR/"
rosparam set /x_min "$x_min"; rosparam set /x_max "$x_max"
rosparam set /y_min "$y_min"; rosparam set /y_max "$y_max"
rosparam set /z_min "$z_min"; rosparam set /z_max "$z_max"

rosrun fast_calib fast_calib >"$OUTPUT_DIR/fast_calib.log" 2>&1 &
calib_pid=$!
deadline=$((SECONDS + 180))
while [[ ! -s "$OUTPUT_DIR/single_calib_result.txt" || ! -s "$OUTPUT_DIR/circle_center_record.txt" ]]; do
  if ! kill -0 "$calib_pid" 2>/dev/null; then
    echo "FAST-Calib exited before producing a result. Log: $OUTPUT_DIR/fast_calib.log" >&2
    exit 1
  fi
  (( SECONDS < deadline )) || {
    echo "FAST-Calib timed out. Adjust the distance crop after inspecting: $OUTPUT_DIR/fast_calib.log" >&2
    exit 1
  }
  sleep 1
done

kill -INT "$calib_pid" 2>/dev/null || true
wait "$calib_pid" 2>/dev/null || true
calib_pid=""
grep -q 'T_cam_lidar' "$OUTPUT_DIR/fast_calib.log" || {
  echo "Result files exist but no T_cam_lidar was logged." >&2
  exit 1
}
echo "Single-scene result: $OUTPUT_DIR/single_calib_result.txt"
echo "Visual check image: $OUTPUT_DIR/qr_detect.png"
echo "Colored cloud: $OUTPUT_DIR/colored_cloud.pcd"
