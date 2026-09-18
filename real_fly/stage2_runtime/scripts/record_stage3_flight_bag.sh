#!/usr/bin/env bash
# Record a Stage-3 flight for post-flight analysis. This script starts/reuses
# only the D435 sensor and rosbag; it never publishes control commands.
set -euo pipefail

usage() {
  echo "Usage: $0 RUN_ID TASK_ID [--no-camera] [--with-cloud]" >&2
}

[[ $# -ge 2 ]] || { usage; exit 2; }
run_id="$1"
task_id="$2"
shift 2
[[ "$run_id" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] || { echo "Invalid RUN_ID" >&2; exit 2; }
[[ "$task_id" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] || { echo "Invalid TASK_ID" >&2; exit 2; }

with_camera=1
with_cloud=0
while (($#)); do
  case "$1" in
    --no-camera) with_camera=0; shift ;;
    --with-cloud) with_cloud=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage; exit 2 ;;
  esac
done

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
runtime_root="$(cd -- "$script_dir/.." && pwd)"
project_root="$(cd -- "$runtime_root/../.." && pwd)"
stage1_root="$project_root/real_fly/stage1_exploration"

# shellcheck disable=SC1091
source "$stage1_root/scripts/env.sh"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
rosnode list >/dev/null 2>&1 || {
  echo "ROS master unavailable; start global localization first." >&2
  exit 1
}

mission="$runtime_root/missions/$run_id/$task_id"
[[ -s "$mission/execution_bundle.json" ]] || {
  echo "Missing execution bundle: $mission/execution_bundle.json" >&2
  exit 1
}

session="flight_$(date +%Y%m%d_%H%M%S)"
output_dir="$runtime_root/runtime/flight_bags/$run_id/$task_id/$session"
mkdir -p "$output_dir/context"
cp -a "$mission/execution_bundle.json" "$output_dir/context/"
[[ ! -s "$mission/task_graph.json" ]] || cp -a "$mission/task_graph.json" "$output_dir/context/"
rosnode list > "$output_dir/rosnode_list.txt"
rostopic list -v > "$output_dir/rostopic_list.txt"

camera_pid=""
cleanup() {
  trap - EXIT INT TERM
  if [[ -n "$camera_pid" ]]; then
    kill -INT "$camera_pid" 2>/dev/null || true
    wait "$camera_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

wait_topic() {
  local topic="$1" label="$2" deadline=$((SECONDS + 25))
  until timeout 3 rostopic echo -n 1 "$topic" >/dev/null 2>&1; do
    ((SECONDS < deadline)) || { echo "Timed out waiting for $label ($topic)" >&2; exit 1; }
    sleep 1
  done
}

camera_topics=()
if [[ "$with_camera" -eq 1 ]]; then
  camera_topics=(
    /camera/color/image_raw
    /camera/color/camera_info
  )
  if rostopic info /camera/color/image_raw 2>/dev/null \
      | sed -n '/^Publishers:/,/^Subscribers:/p' \
      | grep -qE '^ \* /'; then
    echo "Refusing to reuse an existing D435 stream because its source frame rate is unknown." >&2
    echo "Stop the existing camera, or use --no-camera." >&2
    exit 1
  else
    echo "Starting D435 RGB-only at 640x480, source 6 Hz."
    taskset -c 0 nice -n 10 roslaunch "$stage1_root/launch/realsense_d435i.launch" \
      camera_name:=camera enable_color:=true enable_depth:=false \
      enable_accel:=false enable_gyro:=false enable_sync:=false align_depth:=false \
      color_width:=640 color_height:=480 color_fps:=6 \
      > "$output_dir/realsense.log" 2>&1 &
    camera_pid=$!
  fi
  wait_topic /camera/color/image_raw "D435 RGB"
  "$stage1_root/scripts/configure_realsense_rgb.sh"
fi

topics=(
  /ekf_quat/ekf_odom /Odometry /DebugOdometry /LioDebug
  /tf /tf_static
  /planning/click_goal /planning/pos_cmd /planning/super_pos_cmd
  /planning/gate_pos_cmd /planning/gate_active
  /planning/full_smooth_path /planning/full_smooth_markers
  /planning/super_pause /planning_cmd/poly_traj /traj_start_trigger
  /px4ctrl/takeoff_land /debugPx4ctrl
  /pre_map_vln/runtime_status /pre_map_vln/approved_route /pre_map_vln/approved_goals
  /mavros/state /mavros/extended_state /mavros/rc/in
  /mavros/imu/data /mavros/imu/data_raw /mavros/battery
  /mavros/local_position/odom /mavros/setpoint_raw/attitude
  /diagnostics /rosout /rosout_agg
)
topics+=("${camera_topics[@]}")
cloud_topic="${STAGE3_CLOUD_TOPIC:-/cloud_registered}"
if [[ "$with_cloud" -eq 1 ]]; then
  wait_topic "$cloud_topic" "registered online point cloud"
  topics+=("$cloud_topic")
fi

cat > "$output_dir/session.yaml" <<EOF
run_id: $run_id
task_id: $task_id
session: $session
started_at: $(date --iso-8601=seconds)
camera: $([[ "$with_camera" -eq 1 ]] && echo d435_rgb_640x480_6hz || echo disabled)
point_clouds: $([[ "$with_cloud" -eq 1 ]] && echo "$cloud_topic" || echo disabled)
recorder_scheduling: taskset_cpu0_nice10
EOF

echo "Recording Stage-3 flight: $output_dir"
echo "Start this before takeoff/task execution; press Ctrl-C only after landing and disarming."
echo "No arm, takeoff, landing, planner goal, or control command is published by this recorder."
taskset -c 0 nice -n 10 rosbag record --lz4 --tcpnodelay --split --size=2048 \
  --buffsize=512 --chunksize=768 \
  -O "$output_dir/$session" "${topics[@]}"
