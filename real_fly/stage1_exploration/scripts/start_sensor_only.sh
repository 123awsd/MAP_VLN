#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: $0 --run-id ID --master-port PORT (--start-d435i | --start-mid360 --mid360-config FILE)"
  echo "Starts explicitly selected sensor launch files only; it does not start roscore or flight nodes."
}

run_id=""
master_port=""
start_d435i=0
start_mid360=0
mid360_config=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --run-id) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; run_id="$2"; shift 2 ;;
    --master-port) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; master_port="$2"; shift 2 ;;
    --start-d435i) start_d435i=1; shift ;;
    --start-mid360) start_mid360=1; shift ;;
    --mid360-config) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; mid360_config="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ "$run_id" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] || { echo "Invalid run id." >&2; exit 2; }
[[ "$master_port" =~ ^[1-9][0-9]{3,4}$ && "$master_port" -ne 11311 ]] || {
  echo "Pass a non-borrowed isolated ROS master port (not 11311)." >&2
  exit 2
}
[[ "$start_d435i" -eq 1 || "$start_mid360" -eq 1 ]] || {
  echo "Select at least one sensor explicitly." >&2
  exit 2
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STAGE1_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
source "$SCRIPT_DIR/env.sh"

if [[ "$start_mid360" -eq 1 ]]; then
  [[ -n "$mid360_config" && -f "$mid360_config" ]] || {
    echo "MID-360 launch requires an existing run-local JSON config." >&2
    exit 2
  }
  mid360_config="$(readlink -f -- "$mid360_config")"
  case "$mid360_config" in
    "$STAGE1_ROOT"/*) ;;
    *) echo "MID-360 config must be under $STAGE1_ROOT." >&2; exit 2 ;;
  esac
fi

command -v roslaunch >/dev/null 2>&1 || { echo "roslaunch not found; source ROS Noetic first." >&2; exit 1; }
command -v rosparam >/dev/null 2>&1 || { echo "rosparam not found; source ROS Noetic first." >&2; exit 1; }
command -v rospack >/dev/null 2>&1 || { echo "rospack not found; source ROS Noetic first." >&2; exit 1; }

if [[ "$start_d435i" -eq 1 ]]; then
  rospack find realsense2_camera >/dev/null 2>&1 || {
    echo "realsense2_camera is unavailable; install the approved ROS Noetic package first." >&2
    exit 1
  }
fi
if [[ "$start_mid360" -eq 1 ]]; then
  rospack find livox_ros_driver2 >/dev/null 2>&1 || {
    echo "livox_ros_driver2 is unavailable in the isolated overlay." >&2
    exit 1
  }
fi

export ROS_MASTER_URI="http://127.0.0.1:${master_port}"
export ROS_IP="127.0.0.1"
export ROS_HOSTNAME="127.0.0.1"

rosparam list >/dev/null 2>&1 || {
  echo "No reachable ROS master at $ROS_MASTER_URI; start that isolated roscore first." >&2
  exit 1
}

session_dir="$STAGE1_ROOT/runtime/sensor_sessions/$run_id"
[[ ! -e "$session_dir" ]] || { echo "Refusing to overwrite sensor session: $session_dir" >&2; exit 2; }
mkdir -p "$session_dir"
{
  echo "run_id=$run_id"
  echo "started_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "ros_master_uri=$ROS_MASTER_URI"
  echo "workspace=$STAGE1_WS"
  [[ "$start_d435i" -eq 1 ]] && echo "sensor=d435i"
  [[ "$start_mid360" -eq 1 ]] && echo "sensor=mid360"
  [[ "$start_mid360" -eq 1 ]] && echo "mid360_config=$mid360_config"
} > "$session_dir/session.env"

pids=()
cleanup() {
  set +e
  for pid in "${pids[@]:-}"; do
    kill -INT "$pid" 2>/dev/null || true
  done
  for pid in "${pids[@]:-}"; do
    wait "$pid" 2>/dev/null || true
  done
}
on_signal() {
  cleanup
  exit 130
}
trap cleanup EXIT
trap on_signal INT TERM

if [[ "$start_d435i" -eq 1 ]]; then
  echo "Starting D435i sensor-only launch; log: $session_dir/realsense.log"
  roslaunch "$STAGE1_ROOT/launch/realsense_d435i.launch" \
    camera_name:=camera enable_color:=true enable_depth:=true enable_gyro:=true \
    enable_accel:=true enable_sync:=true align_depth:=true \
    > "$session_dir/realsense.log" 2>&1 &
  pids+=("$!")
fi

if [[ "$start_mid360" -eq 1 ]]; then
  echo "Starting MID-360 sensor-only launch; log: $session_dir/livox.log"
  roslaunch "$STAGE1_ROOT/launch/livox_mid360.launch" \
    config:="$mid360_config" \
    > "$session_dir/livox.log" 2>&1 &
  pids+=("$!")
fi

echo "Sensor launches are running under $ROS_MASTER_URI. Press Ctrl-C to stop only these child launches."
while :; do
  for pid in "${pids[@]}"; do
    if ! kill -0 "$pid" 2>/dev/null; then
      echo "A sensor launch exited unexpectedly: pid=$pid" >&2
      exit 1
    fi
  done
  sleep 1
done
