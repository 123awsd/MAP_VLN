#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: run_mapping_from_bag.sh --bag FILE --run-id ID --runtime-root DIR [options]

Options:
  --output-dir DIR          Output directory (default: stage1 data/ID/mapping)
  --rate RATE               rosbag playback rate (default: 0.5)
  --drain-timeout SEC       Maximum wall time for the output queue to drain (default: 180)
  --coverage-tolerance SEC  Allowed final lidar/odometry stamp gap (default: 0.25)
  --master-port PORT        Isolated ROS master port (default: 11312; never 11311)

Only /livox/lidar and /mavros/imu/data are replayed. Existing derived odometry,
registered clouds and TF in the input bag are never fed to the mapper.
EOF
}

bag=""
run_id=""
runtime_root=""
output_dir=""
rate="0.5"
drain_timeout="180"
coverage_tolerance="0.25"
port=11312
while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; bag="$2"; shift 2 ;;
    --run-id) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; run_id="$2"; shift 2 ;;
    --runtime-root) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; runtime_root="$2"; shift 2 ;;
    --output-dir) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; output_dir="$2"; shift 2 ;;
    --rate) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; rate="$2"; shift 2 ;;
    --drain-timeout) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; drain_timeout="$2"; shift 2 ;;
    --coverage-tolerance) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; coverage_tolerance="$2"; shift 2 ;;
    --master-port) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; port="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ -f "$bag" ]] || { echo "Bag not found: $bag" >&2; exit 2; }
[[ "$run_id" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] || { echo "Invalid run id." >&2; exit 2; }
[[ "$rate" =~ ^(0|[1-9][0-9]*)(\.[0-9]+)?$ ]] || { echo "Invalid playback rate: $rate" >&2; exit 2; }
[[ "$drain_timeout" =~ ^[1-9][0-9]*$ ]] || { echo "Invalid drain timeout: $drain_timeout" >&2; exit 2; }
[[ "$coverage_tolerance" =~ ^(0|[1-9][0-9]*)(\.[0-9]+)?$ ]] || { echo "Invalid coverage tolerance." >&2; exit 2; }
[[ "$port" =~ ^[1-9][0-9]{3,4}$ && "$port" -ne 11311 ]] || { echo "Use a free non-borrowed ROS master port." >&2; exit 2; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STAGE1_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_ROOT="$(cd "$STAGE1_ROOT/../.." && pwd)"
source "$SCRIPT_DIR/env.sh"

[[ -n "$runtime_root" ]] || { echo "An explicit per-run --runtime-root is required." >&2; exit 2; }
runtime_root="$(realpath -m "$runtime_root")"
case "$runtime_root" in
  "$STAGE1_ROOT"/*) ;;
  *) echo "Runtime root must be under $STAGE1_ROOT." >&2; exit 2 ;;
esac
[[ -d "$runtime_root/Log" && -d "$runtime_root/PCD" ]] || {
  echo "Runtime root is not prepared; run build_stage1_fast_lio.sh with this exact root first." >&2
  exit 1
}

if [[ -z "$output_dir" ]]; then
  output_dir="$STAGE1_ROOT/data/$run_id/mapping"
fi
output_dir="$(realpath -m "$output_dir")"
case "$output_dir" in
  "$REPO_ROOT"/*) ;;
  *) echo "Output directory must be inside the current repository." >&2; exit 2 ;;
esac

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

mkdir -p "$output_dir"
config="$STAGE1_ROOT/config/fast_lio_mid360_handheld.yaml"
output_bag="$output_dir/mapping_outputs.bag"
# Keep the original iKD-tree dump as a diagnostic, but use FAST-LIO's native
# complete scan map as the canonical offline map. The live configuration keeps
# pcd_save disabled to protect real-time performance; this script explicitly
# enables it only for offline replay.
runtime_kdtree_map="$runtime_root/PCD/handheld_map_${run_id}.pcd"
runtime_native_map="$runtime_root/PCD/scans.pcd"
output_map="$output_dir/handheld_map_${run_id}.pcd"
progress_file="$output_dir/mapping_progress.json"
audit_file="$output_dir/fastlio_audit.json"
for existing_output in \
  "$output_bag" "$output_bag.active" "$runtime_kdtree_map" "$runtime_native_map" "$output_map" "$progress_file" "$audit_file" \
  "$output_dir/fastlio.log" "$output_dir/roscore.log" \
  "$output_dir/rosbag_record.log" "$output_dir/rosbag_play.log" "$output_dir/progress_monitor.log"; do
  [[ ! -e "$existing_output" ]] || { echo "Refusing to overwrite existing output: $existing_output" >&2; exit 2; }
done

expected_last_lidar="$(python3 "$SCRIPT_DIR/audit_fastlio_output.py" \
  --input-bag "$bag" --last-input-only)"
echo "Expected final lidar header stamp: $expected_last_lidar"

export ROS_MASTER_URI="http://127.0.0.1:${port}"
export ROS_IP="127.0.0.1"
export ROS_HOSTNAME="127.0.0.1"
export ROS_TIME_USE_SYSTEM_TIME="0"

core_pid=""
map_pid=""
record_pid=""
play_pid=""
monitor_pid=""
cleanup() {
  set +e
  [[ -n "$play_pid" ]] && kill -INT "$play_pid" 2>/dev/null || true
  [[ -n "$monitor_pid" ]] && kill -TERM "$monitor_pid" 2>/dev/null || true
  [[ -n "$map_pid" ]] && kill -INT "$map_pid" 2>/dev/null || true
  [[ -n "$record_pid" ]] && kill -INT "$record_pid" 2>/dev/null || true
  [[ -n "$core_pid" ]] && kill -INT "$core_pid" 2>/dev/null || true
  [[ -n "$play_pid" ]] && wait "$play_pid" 2>/dev/null || true
  [[ -n "$monitor_pid" ]] && wait "$monitor_pid" 2>/dev/null || true
  [[ -n "$map_pid" ]] && wait "$map_pid" 2>/dev/null || true
  [[ -n "$record_pid" ]] && wait "$record_pid" 2>/dev/null || true
  [[ -n "$core_pid" ]] && wait "$core_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "Starting isolated offline ROS master on ${ROS_MASTER_URI}"
roscore -p "$port" > "$output_dir/roscore.log" 2>&1 &
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
# Native FAST-LIO complete-map export is enabled only in this offline mapper.
# It stores the registered scans with their original x/y/z/intensity fields.
rosparam set pcd_save/pcd_save_en true
rosparam set pcd_save/interval -1
rosparam set wxx/new_map_pcd_name "handheld_map_${run_id}.pcd"
echo "Starting only the Stage-1 FAST-LIO mapper"
"$binary" > "$output_dir/fastlio.log" 2>&1 &
map_pid=$!
sleep 3
kill -0 "$map_pid" 2>/dev/null || { echo "FAST-LIO exited during startup; inspect fastlio.log." >&2; exit 1; }

python3 "$SCRIPT_DIR/monitor_mapping_progress.py" --output "$progress_file" \
  > "$output_dir/progress_monitor.log" 2>&1 &
monitor_pid=$!

echo "Recording regenerated mapper outputs"
rosbag record --lz4 -O "$output_bag" \
  /Odometry /cloud_registered /cloud_registered_body /LioDebug /tf /tf_static \
  > "$output_dir/rosbag_record.log" 2>&1 &
record_pid=$!
sleep 2

echo "Replaying only raw lidar and flight-controller IMU at ${rate}x"
rosbag play --clock --rate "$rate" "$bag" \
  --topics /livox/lidar /mavros/imu/data \
  > "$output_dir/rosbag_play.log" 2>&1 &
play_pid=$!
wait "$play_pid"
play_pid=""

echo "Playback ended; waiting for FAST-LIO's output queue to reach the final lidar stamp"
drain_started="$(date +%s)"
last_odom=""
while true; do
  kill -0 "$map_pid" 2>/dev/null || { echo "FAST-LIO exited before queue drain completed." >&2; exit 1; }
  if [[ -f "$progress_file" ]]; then
    last_odom="$(python3 -c 'import json,sys; value=json.load(open(sys.argv[1])).get("last_header_stamp"); print("" if value is None else "%.9f" % value)' "$progress_file")"
  fi
  if [[ -n "$last_odom" ]] && awk -v actual="$last_odom" -v expected="$expected_last_lidar" -v tolerance="$coverage_tolerance" \
      'BEGIN { exit !(actual >= expected - tolerance) }'; then
    echo "Queue drained: final odometry stamp ${last_odom}"
    break
  fi
  now="$(date +%s)"
  if (( now - drain_started >= drain_timeout )); then
    echo "Timed out waiting for queue drain; last odometry=${last_odom:-none}, expected=${expected_last_lidar}." >&2
    exit 1
  fi
  sleep 1
done
sleep 2

kill -TERM "$monitor_pid" 2>/dev/null || true
wait "$monitor_pid" 2>/dev/null || true
monitor_pid=""
kill -INT "$map_pid" 2>/dev/null || true
wait "$map_pid" 2>/dev/null || true
map_pid=""
kill -INT "$record_pid" 2>/dev/null || true
wait "$record_pid" 2>/dev/null || true
record_pid=""

[[ -s "$runtime_native_map" ]] || {
  echo "FAST-LIO did not produce its native complete map: $runtime_native_map" >&2
  exit 1
}
cp --reflink=auto "$runtime_native_map" "$output_map"
if [[ -s "$runtime_kdtree_map" ]]; then
  cp --reflink=auto "$runtime_kdtree_map" "$output_dir/handheld_map_${run_id}_kdtree_local.pcd"
fi
python3 "$SCRIPT_DIR/audit_fastlio_output.py" \
  --input-bag "$bag" \
  --output-bag "$output_bag" \
  --map-pcd "$output_map" \
  --json "$audit_file"

echo "Offline mapping output: $output_dir"
echo "Output bag: $output_bag"
echo "Native FAST-LIO map PCD: $output_map"
[[ -s "$output_dir/handheld_map_${run_id}_kdtree_local.pcd" ]] && \
  echo "Diagnostic local iKD-tree PCD: $output_dir/handheld_map_${run_id}_kdtree_local.pcd"
echo "Audit: $audit_file"
