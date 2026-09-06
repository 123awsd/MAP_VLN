#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: $0 --confirm-handheld --run-id ID --topics-config /path/topics.env [--duration-sec N]"
}

confirm=0
run_id=""
topics_config=""
duration=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --confirm-handheld) confirm=1; shift ;;
    --run-id) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; run_id="$2"; shift 2 ;;
    --topics-config) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; topics_config="$2"; shift 2 ;;
    --duration-sec) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; duration="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ "$confirm" -eq 1 ]] || { echo "Recording is armed only with --confirm-handheld." >&2; exit 2; }
[[ "$run_id" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] || { echo "Invalid run id." >&2; exit 2; }
[[ -f "$topics_config" ]] || { echo "Topic config not found." >&2; exit 2; }
if [[ -n "$duration" && ! "$duration" =~ ^[1-9][0-9]*$ ]]; then
  echo "Duration must be a positive integer number of seconds." >&2
  exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STAGE1_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
source "$SCRIPT_DIR/env.sh"
[[ "${ROS_MASTER_URI:-}" != *:11311* ]] || { echo "Refusing borrowed ROS master ${ROS_MASTER_URI:-unset}." >&2; exit 2; }
command -v rosbag >/dev/null 2>&1 || { echo "rosbag not found." >&2; exit 1; }
"$SCRIPT_DIR/check_ros_topics.sh" --config "$topics_config" --require-rgb
"$SCRIPT_DIR/configure_realsense_rgb.sh"

value() {
  /usr/bin/awk -F= -v key="$1" '$1 == key { sub(/^[ \t]+/, "", $2); sub(/[ \t]+$/, "", $2); print $2; exit }' "$topics_config"
}
unsafe() {
  echo "$1" | /usr/bin/grep -Eiq 'cmd_vel|setpoint|position_command|trajectory|mavros|flight|motor|takeoff|offboard|px4|ardupilot|mavlink|habitat|uav_simulator|sensor_pose'
}

topics=()
for key in LIDAR_TOPIC IMU_TOPIC RGB_TOPIC DEPTH_TOPIC RGB_CAMERA_INFO_TOPIC DEPTH_CAMERA_INFO_TOPIC \
  CAMERA_IMU_TOPIC CAMERA_GYRO_TOPIC CAMERA_ACCEL_TOPIC TF_TOPIC TF_STATIC_TOPIC; do
  topic="$(value "$key")"
  [[ -n "$topic" ]] || continue
  if unsafe "$topic"; then
    echo "unsafe topic refused: $topic" >&2
    exit 2
  fi
  duplicate=0
  for existing in "${topics[@]}"; do
    [[ "$existing" == "$topic" ]] && duplicate=1
  done
  [[ "$duplicate" -eq 0 ]] && topics+=("$topic")
done
[[ "${#topics[@]}" -ge 6 ]] || { echo "Too few verified sensor topics." >&2; exit 2; }

run_dir="$STAGE1_ROOT/data/$run_id"
[[ ! -e "$run_dir" ]] || { echo "Refusing to overwrite existing run: $run_dir" >&2; exit 2; }
mkdir -p "$run_dir/raw"
manifest="$run_dir/manifest.txt"
{
  echo "run_id=$run_id"
  echo "started_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "ros_master_uri=$ROS_MASTER_URI"
  echo "topics_config=$topics_config"
  echo "rgb_profile=$STAGE1_ROOT/config/realsense_d435_recording.conf"
  echo "rgb_auto_exposure=false"
  echo "rgb_exposure=120"
  for topic in "${topics[@]}"; do
    echo "topic=$topic"
  done
} > "$manifest"

bag="$run_dir/raw/sensors.bag"
record_args=(rosbag record --lz4 -O "$bag" "${topics[@]}")
echo "Recording sensor topics to $bag"
echo "Stop with Ctrl-C while the device is stationary; no motion command is sent."
if [[ -n "$duration" ]]; then
  set +e
  timeout --signal=INT "${duration}s" "${record_args[@]}"
  rc=$?
  set -e
  [[ "$rc" -eq 0 || "$rc" -eq 124 || "$rc" -eq 130 ]] || exit "$rc"
else
  "${record_args[@]}"
fi

echo "ended_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)" >> "$manifest"
echo "Bag written: $bag"
