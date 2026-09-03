#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: $0 --bag /path/raw/sensors.bag --run-id ID --runtime-root /path/under/stage1 [--master-port PORT]"
}

bag=""
run_id=""
runtime_root=""
port=11312
while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; bag="$2"; shift 2 ;;
    --run-id) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; run_id="$2"; shift 2 ;;
    --runtime-root) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; runtime_root="$2"; shift 2 ;;
    --master-port) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; port="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ -f "$bag" ]] || { echo "Bag not found: $bag" >&2; exit 2; }
[[ "$run_id" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] || { echo "Invalid run id." >&2; exit 2; }
[[ "$port" =~ ^[1-9][0-9]{3,4}$ && "$port" -ne 11311 ]] || { echo "Use a free non-borrowed ROS master port." >&2; exit 2; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STAGE1_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
source "$SCRIPT_DIR/env.sh"
[[ -n "$runtime_root" ]] || { echo "An explicit per-run --runtime-root is required." >&2; exit 2; }
case "$runtime_root" in
  "$STAGE1_ROOT"/*) ;;
  *) echo "Runtime root must be under $STAGE1_ROOT." >&2; exit 2 ;;
esac
[[ -d "$runtime_root/Log" && -d "$runtime_root/PCD" ]] || {
  echo "Runtime root is not prepared; run build_stage1_fast_lio.sh with this exact root first." >&2
  exit 1
}
command -v roscore >/dev/null 2>&1 || { echo "roscore not found." >&2; exit 1; }
command -v rosparam >/dev/null 2>&1 || { echo "rosparam not found." >&2; exit 1; }
command -v rosbag >/dev/null 2>&1 || { echo "rosbag not found." >&2; exit 1; }
[[ -x "$STAGE1_ROOT/ros_ws/devel/lib/stage1_fast_lio/fastlio_mapping" ]] || {
  echo "Stage-1 FAST-LIO binary missing; run build_stage1_fast_lio.sh first." >&2
  exit 1
}
binary="$STAGE1_ROOT/ros_ws/devel/lib/stage1_fast_lio/fastlio_mapping"
strings "$binary" | /usr/bin/grep -F "$runtime_root/" >/dev/null || {
  echo "FAST-LIO binary ROOT_DIR does not match $runtime_root; rebuild with this exact root." >&2
  exit 1
}

run_dir="$STAGE1_ROOT/data/$run_id"
[[ -d "$run_dir" ]] || { echo "Run directory not found: $run_dir" >&2; exit 2; }
mkdir -p "$run_dir/mapping"
config="$STAGE1_ROOT/config/fast_lio_mid360_handheld.yaml"
output_bag="$run_dir/mapping/mapping_outputs.bag"
map_pcd="$runtime_root/PCD/handheld_map_${run_id}.pcd"
[[ ! -e "$map_pcd" ]] || { echo "Refusing to overwrite existing map: $map_pcd" >&2; exit 2; }
for existing_output in "$output_bag" "$run_dir/mapping/fastlio.log" "$run_dir/mapping/roscore.log" "$run_dir/mapping/rosbag_record.log" "$run_dir/mapping/rosbag_play.log"; do
  [[ ! -e "$existing_output" ]] || { echo "Refusing to overwrite existing output: $existing_output" >&2; exit 2; }
done

export ROS_MASTER_URI="http://127.0.0.1:${port}"
export ROS_IP="127.0.0.1"
export ROS_HOSTNAME="127.0.0.1"
export ROS_TIME_USE_SYSTEM_TIME="0"

core_pid=""
map_pid=""
record_pid=""
play_pid=""
cleanup() {
  set +e
  [[ -n "$play_pid" ]] && kill -INT "$play_pid" 2>/dev/null || true
  [[ -n "$record_pid" ]] && kill -INT "$record_pid" 2>/dev/null || true
  [[ -n "$map_pid" ]] && kill -INT "$map_pid" 2>/dev/null || true
  [[ -n "$core_pid" ]] && kill -INT "$core_pid" 2>/dev/null || true
  [[ -n "$play_pid" ]] && wait "$play_pid" 2>/dev/null || true
  [[ -n "$record_pid" ]] && wait "$record_pid" 2>/dev/null || true
  [[ -n "$map_pid" ]] && wait "$map_pid" 2>/dev/null || true
  [[ -n "$core_pid" ]] && wait "$core_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "Starting isolated offline ROS master on ${ROS_MASTER_URI}"
roscore -p "$port" > "$run_dir/mapping/roscore.log" 2>&1 &
core_pid=$!
for _ in $(seq 1 30); do
  if rosparam list >/dev/null 2>&1; then
    break
  fi
  sleep 1
done
rosparam list >/dev/null 2>&1 || { echo "isolated roscore did not become ready." >&2; exit 1; }

rosparam load "$config"
rosparam set use_sim_time true
rosparam set wxx/new_map_pcd_name "handheld_map_${run_id}.pcd"
echo "Starting only the Stage-1 FAST-LIO mapper"
"$binary" > "$run_dir/mapping/fastlio.log" 2>&1 &
map_pid=$!
sleep 3

echo "Recording offline mapper outputs"
rosbag record --lz4 -O "$output_bag" \
  /Odometry /cloud_registered /cloud_registered_body /LioDebug /tf /tf_static \
  > "$run_dir/mapping/rosbag_record.log" 2>&1 &
record_pid=$!
sleep 2

echo "Replaying input bag; no sensor driver or flight node is started"
rosbag play --clock --rate 1.0 "$bag" > "$run_dir/mapping/rosbag_play.log" 2>&1 &
play_pid=$!
wait "$play_pid"
play_pid=""
sleep 2

kill -INT "$record_pid" 2>/dev/null || true
wait "$record_pid" 2>/dev/null || true
record_pid=""
kill -INT "$map_pid" 2>/dev/null || true
wait "$map_pid" 2>/dev/null || true
map_pid=""

[[ -f "$map_pcd" ]] || { echo "FAST-LIO did not produce the expected map: $map_pcd" >&2; exit 1; }
echo "Offline mapping output: $run_dir/mapping"
echo "Map PCD: $map_pcd"
