#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: $0 --config /path/to/topics.env [--duration-sec N]"
}

config=""
duration=5
while [[ $# -gt 0 ]]; do
  case "$1" in
    --config) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; config="$2"; shift 2 ;;
    --duration-sec) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; duration="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done
[[ -f "$config" ]] || { echo "Topic config not found: $config" >&2; exit 2; }
[[ "$duration" =~ ^[1-9][0-9]*$ ]] || { echo "Duration must be a positive integer." >&2; exit 2; }
[[ "${ROS_MASTER_URI:-}" != *:11311* && -n "${ROS_MASTER_URI:-}" ]] || {
  echo "Use an explicit isolated ROS master, not the borrowed port 11311." >&2
  exit 2
}

value() {
  /usr/bin/awk -F= -v key="$1" '$1 == key { sub(/^[ \t]+/, "", $2); sub(/[ \t]+$/, "", $2); print $2; exit }' "$config"
}
unsafe() {
  echo "$1" | /usr/bin/grep -Eiq 'cmd_vel|setpoint|position_command|trajectory|mavros|flight|motor|takeoff|offboard|px4|ardupilot|mavlink|habitat|uav_simulator|sensor_pose'
}

failed=0
for item in 'lidar|LIDAR_TOPIC' 'imu|IMU_TOPIC' 'rgb|RGB_TOPIC' 'depth|DEPTH_TOPIC'; do
  IFS='|' read -r label key <<< "$item"
  topic="$(value "$key")"
  if [[ -z "$topic" ]]; then
    echo "missing $label topic" >&2
    failed=1
    continue
  fi
  if unsafe "$topic"; then
    echo "unsafe topic refused: $topic" >&2
    failed=1
    continue
  fi
  echo "--- $label: $topic (${duration}s) ---"
  set +e
  output="$(timeout --signal=INT "${duration}s" rostopic hz "$topic" 2>&1)"
  rc=$?
  set -e
  echo "$output"
  if [[ "$rc" -ne 0 && "$rc" -ne 124 && "$rc" -ne 130 ]] || ! echo "$output" | /usr/bin/grep -q 'average rate'; then
    echo "no measurable rate for $topic" >&2
    failed=1
  fi
done

camera_imu="$(value CAMERA_IMU_TOPIC)"
if [[ -n "$camera_imu" ]]; then
  camera_items=("camera_imu|CAMERA_IMU_TOPIC")
else
  camera_items=("camera_gyro|CAMERA_GYRO_TOPIC" "camera_accel|CAMERA_ACCEL_TOPIC")
fi
for item in "${camera_items[@]}"; do
  IFS='|' read -r label key <<< "$item"
  topic="$(value "$key")"
  if [[ -z "$topic" ]]; then
    echo "missing $label topic" >&2
    failed=1
    continue
  fi
  if unsafe "$topic"; then
    echo "unsafe topic refused: $topic" >&2
    failed=1
    continue
  fi
  echo "--- $label: $topic (${duration}s) ---"
  set +e
  output="$(timeout --signal=INT "${duration}s" rostopic hz "$topic" 2>&1)"
  rc=$?
  set -e
  echo "$output"
  if [[ "$rc" -ne 0 && "$rc" -ne 124 && "$rc" -ne 130 ]] || ! echo "$output" | /usr/bin/grep -q 'average rate'; then
    echo "no measurable rate for $topic" >&2
    failed=1
  fi
done

[[ "$failed" -eq 0 ]] || exit 1
