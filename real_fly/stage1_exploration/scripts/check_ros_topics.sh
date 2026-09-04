#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: $0 --config /path/to/topics.env [--require-rgb]"
}

config=""
require_rgb=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --config) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; config="$2"; shift 2 ;;
    --require-rgb) require_rgb=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done
[[ -f "$config" ]] || { echo "Topic config not found: $config" >&2; exit 2; }

value() {
  /usr/bin/awk -F= -v key="$1" '$1 == key { sub(/^[ \t]+/, "", $2); sub(/[ \t]+$/, "", $2); print $2; exit }' "$config"
}

is_unsafe_topic() {
  echo "$1" | /usr/bin/grep -Eiq 'cmd_vel|setpoint|position_command|trajectory|mavros|flight|motor|takeoff|offboard|px4|ardupilot|mavlink|habitat|uav_simulator|sensor_pose'
}

check_one() {
  local label="$1" key="$2" type_key="$3" topic expected actual
  topic="$(value "$key")"
  [[ -n "$topic" ]] || { echo "missing $label ($key)" >&2; return 1; }
  [[ "$topic" == /* ]] || { echo "$label topic must be absolute: $topic" >&2; return 1; }
  if is_unsafe_topic "$topic"; then
    echo "refusing unsafe/control-like topic for $label: $topic" >&2
    return 1
  fi
  actual="$(rostopic type "$topic" 2>/dev/null || true)"
  [[ -n "$actual" ]] || { echo "$label unavailable: $topic" >&2; return 1; }
  expected="$(value "$type_key")"
  if [[ -n "$expected" && "$actual" != "$expected" ]]; then
    echo "$label type mismatch: $topic (expected $expected, got $actual)" >&2
    return 1
  fi
  printf '%-20s %-48s %s\n' "$label" "$topic" "$actual"
  return 0
}

if [[ "${ROS_MASTER_URI:-}" == *:11311* ]]; then
  echo "Refusing borrowed ROS master ${ROS_MASTER_URI:-unset}; use the Stage-1 isolated master." >&2
  exit 2
fi
command -v rostopic >/dev/null 2>&1 || { echo "rostopic not found; source ROS Noetic first." >&2; exit 1; }

echo "ROS master: ${ROS_MASTER_URI:-unset}"
echo "label                topic                                            type"
failed=0
check_one lidar LIDAR_TOPIC LIDAR_TYPE || failed=1
check_one imu IMU_TOPIC IMU_TYPE || failed=1

optional=(
  "rgb|RGB_TOPIC|RGB_TYPE"
  "depth|DEPTH_TOPIC|DEPTH_TYPE"
  "rgb_camera_info|RGB_CAMERA_INFO_TOPIC|RGB_CAMERA_INFO_TYPE"
  "depth_camera_info|DEPTH_CAMERA_INFO_TOPIC|DEPTH_CAMERA_INFO_TYPE"
  "tf|TF_TOPIC|TF_TYPE"
  "tf_static|TF_STATIC_TOPIC|TF_STATIC_TYPE"
)
for item in "${optional[@]}"; do
  IFS='|' read -r label key type_key <<< "$item"
  topic="$(value "$key")"
  if [[ -n "$topic" ]]; then
    check_one "$label" "$key" "$type_key" || failed=1
  elif [[ "$require_rgb" -eq 1 ]]; then
    echo "missing required $label ($key)" >&2
    failed=1
  fi
done

camera_imu="$(value CAMERA_IMU_TOPIC)"
camera_gyro="$(value CAMERA_GYRO_TOPIC)"
camera_accel="$(value CAMERA_ACCEL_TOPIC)"
camera_imu_required="$(value CAMERA_IMU_REQUIRED)"
if [[ -n "$camera_imu" ]]; then
  check_one camera_imu CAMERA_IMU_TOPIC CAMERA_IMU_TYPE || failed=1
elif [[ "$require_rgb" -eq 1 && "$camera_imu_required" == "1" ]]; then
  [[ -n "$camera_gyro" ]] || { echo "missing D435i gyro topic (CAMERA_GYRO_TOPIC)" >&2; failed=1; }
  [[ -n "$camera_accel" ]] || { echo "missing D435i accel topic (CAMERA_ACCEL_TOPIC)" >&2; failed=1; }
  if [[ -n "$camera_gyro" ]]; then
    check_one camera_gyro CAMERA_GYRO_TOPIC CAMERA_GYRO_TYPE || failed=1
  fi
  if [[ -n "$camera_accel" ]]; then
    check_one camera_accel CAMERA_ACCEL_TOPIC CAMERA_ACCEL_TYPE || failed=1
  fi
else
  [[ -z "$camera_gyro" ]] || check_one camera_gyro CAMERA_GYRO_TOPIC CAMERA_GYRO_TYPE || true
  [[ -z "$camera_accel" ]] || check_one camera_accel CAMERA_ACCEL_TOPIC CAMERA_ACCEL_TYPE || true
fi

[[ "$failed" -eq 0 ]] || exit 1
