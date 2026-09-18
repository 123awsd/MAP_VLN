#!/usr/bin/env bash
set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
  cat <<'EOF'
Usage:
  ./fly.sh mapping
  ./fly.sh global MAP.pcd
  ./fly.sh global MAP.pcd X Y Z YAW_DEG

No argument is equivalent to "mapping" for backward compatibility.
EOF
}

FLY_ARGS=("$@")
if (( $# == 0 )); then
  FLY_ARGS=(mapping)
elif [[ "$1" == "-h" || "$1" == "--help" ]]; then
  usage
  exit 0
elif [[ "$1" == "mapping" ]]; then
  (( $# == 1 )) || { usage >&2; exit 2; }
elif [[ "$1" == "global" ]]; then
  (( $# == 2 || $# == 6 )) || { usage >&2; exit 2; }
  if (( $# == 6 )); then
    for value in "$3" "$4" "$5" "$6"; do
      if [[ ! "${value}" =~ ^[+-]?([0-9]+([.][0-9]*)?|[.][0-9]+)([eE][+-]?[0-9]+)?$ ]]; then
        echo "[fly] ERROR: X Y Z YAW_DEG must be numbers" >&2
        exit 2
      fi
    done
  fi
  MAP_NAME="$2"
  [[ "${MAP_NAME}" == *.pcd ]] || MAP_NAME="${MAP_NAME}.pcd"
  if [[ "${MAP_NAME}" = /* ]]; then
    MAP_PATH="${MAP_NAME}"
  elif [[ "${MAP_NAME}" == */* ]]; then
    echo "[fly] ERROR: relative map must be a filename in ${WS_DIR}" >&2
    exit 3
  elif [[ -r "${WS_DIR}/${MAP_NAME}" ]]; then
    MAP_PATH="${WS_DIR}/${MAP_NAME}"
  else
    MAP_PATH="${WS_DIR}/src/localization/FAST_LIO/PCD/${MAP_NAME}"
  fi
  if [[ ! -r "${MAP_PATH}" ]]; then
    echo "[fly] ERROR: map not found: ${WS_DIR}/${MAP_NAME}" >&2
    exit 3
  fi
else
  echo "[fly] ERROR: mode must be mapping or global" >&2
  usage >&2
  exit 2
fi

# ROS/catkin setup scripts inspect positional arguments. The validated CLI is
# already stored in FLY_ARGS, so clear $@ before sourcing them.
set --
# Load drone/lidar env from /etc/uav/uav.env (bash cannot read ~/.zshrc)
source "${WS_DIR}/scripts/load_uav_env.sh"
source /opt/ros/noetic/setup.bash
source "${WS_DIR}/devel/setup.bash"

# 如串口权限不足，请在启动前单独执行：sudo chmod 666 /dev/ttyTHS0

MAVROS_PID=""
LOCALIZATION_PID=""

cleanup() {
  local rc=$?
  local pid
  trap - EXIT INT TERM
  for pid in "${LOCALIZATION_PID}" "${MAVROS_PID}"; do
    [[ -z "${pid}" ]] || kill "${pid}" 2>/dev/null || true
  done
  for pid in "${LOCALIZATION_PID}" "${MAVROS_PID}"; do
    [[ -z "${pid}" ]] || wait "${pid}" 2>/dev/null || true
  done
  exit "${rc}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

roslaunch mavros px4.launch & MAVROS_PID=$!

MAVROS_SERVICE_TIMEOUT="${MAVROS_SERVICE_TIMEOUT:-20}"
if [[ ! "${MAVROS_SERVICE_TIMEOUT}" =~ ^[1-9][0-9]*$ ]]; then
  echo "[fly] ERROR: MAVROS_SERVICE_TIMEOUT must be a positive integer" >&2
  exit 2
fi
MAVCMD_TIMEOUT="${MAVCMD_TIMEOUT:-10}"
if [[ ! "${MAVCMD_TIMEOUT}" =~ ^[1-9][0-9]*$ ]]; then
  echo "[fly] ERROR: MAVCMD_TIMEOUT must be a positive integer" >&2
  exit 2
fi

deadline=$((SECONDS + MAVROS_SERVICE_TIMEOUT))
while ! rosservice info /mavros/cmd/command >/dev/null 2>&1; do
  if ! kill -0 "${MAVROS_PID}" 2>/dev/null; then
    set +e
    wait "${MAVROS_PID}"
    rc=$?
    set -e
    echo "[fly] ERROR: MAVROS exited before /mavros/cmd/command became available (rc=${rc})" >&2
    (( rc != 0 )) || rc=1
    exit "${rc}"
  fi
  if (( SECONDS >= deadline )); then
    echo "[fly] ERROR: /mavros/cmd/command unavailable after ${MAVROS_SERVICE_TIMEOUT}s" >&2
    exit 4
  fi
  sleep 0.2
done

configure_mavlink_stream() {
  local message_id="$1"
  local message_name="$2"
  # A MAVROS service may exist before the FCU responds. Bound each one-shot
  # command so a missing FCU acknowledgement cannot block localization.
  if ! timeout "${MAVCMD_TIMEOUT}" rosrun mavros mavcmd long 511 "${message_id}" 5000 0 0 0 0 0; then
    echo "[fly] WARN: failed to configure ${message_name} (${message_id})" >&2
  fi
  sleep 0.5
}

# Run these one-shot commands in the foreground. Leaving them as background
# jobs can make Bash 5.0's wait -n return before either long-running stack dies.
configure_mavlink_stream 31 ATTITUDE_QUATERNION
configure_mavlink_stream 105 HIGHRES_IMU
configure_mavlink_stream 83 ATTITUDE_TARGET
configure_mavlink_stream 36 SERVO_OUTPUT_RAW
configure_mavlink_stream 147 BATTERY_STATUS

bash "${WS_DIR}/localization.sh" "${FLY_ARGS[@]}" & LOCALIZATION_PID=$!

# Avoid wait -n here for compatibility with the Bash version on Ubuntu 20.04.
# Both children are reaped by cleanup after either long-running stack exits.
set +e
while kill -0 "${MAVROS_PID}" 2>/dev/null && \
      kill -0 "${LOCALIZATION_PID}" 2>/dev/null; do
  sleep 1
done

if ! kill -0 "${MAVROS_PID}" 2>/dev/null; then
  wait "${MAVROS_PID}"
  rc=$?
  component="MAVROS"
else
  wait "${LOCALIZATION_PID}"
  rc=$?
  component="localization"
fi
set -e

echo "[fly] ERROR: ${component} exited unexpectedly (rc=${rc})" >&2
(( rc != 0 )) || rc=1
exit "${rc}"
