#!/usr/bin/env bash
# Start the complete LiDAR localization chain.
#
# CLI:
#   localization.sh mapping
#   localization.sh global MAP.pcd [X Y Z YAW_DEG]
# With no arguments, environment variables remain available for GCS/systemd.
set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOCALIZATION_ARGS=("$@")
# ROS/catkin setup scripts inspect positional arguments. Do not let our CLI
# flags leak into them; restore the arguments before parsing below.
set --
source "${WS_DIR}/scripts/load_uav_env.sh"
source /opt/ros/noetic/setup.bash
source "${WS_DIR}/devel/setup.bash"
set -- "${LOCALIZATION_ARGS[@]}"
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./localization.sh mapping
  ./localization.sh global MAP.pcd
  ./localization.sh global MAP.pcd X Y Z YAW_DEG

Relative map names are read from the dls_ws root. If no initial pose is
given, global mode uses 0 0 0 0. With no arguments, FAST_LIO_MODE and the
GLOBAL_MAP_*/INIT_BODY_* environment variables are used (GCS compatibility).
EOF
}

is_number() {
  [[ "$1" =~ ^[+-]?([0-9]+([.][0-9]*)?|[.][0-9]+)([eE][+-]?[0-9]+)?$ ]]
}

CLI_SELECTED=false
if (( $# > 0 )); then
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    mapping)
      (( $# == 1 )) || { usage >&2; exit 2; }
      MODE="mapping"
      ;;
    global)
      (( $# == 2 || $# == 6 )) || { usage >&2; exit 2; }
      MODE="global"
      MAP_VALUE="$2"
      CLI_SELECTED=true
      if (( $# == 6 )); then
        for value in "$3" "$4" "$5" "$6"; do
          is_number "${value}" || {
            echo "[localization] ERROR: initial pose values must be numbers" >&2
            exit 2
          }
        done
        INIT_BODY_X="$3"
        INIT_BODY_Y="$4"
        INIT_BODY_Z="$5"
        INIT_BODY_YAW_DEG="$6"
      else
        INIT_BODY_X=0.0
        INIT_BODY_Y=0.0
        INIT_BODY_Z=0.0
        INIT_BODY_YAW_DEG=0.0
        echo "[localization] WARN: no initial pose supplied; assuming map origin (0, 0, 0, yaw 0)" >&2
      fi
      export INIT_BODY_X INIT_BODY_Y INIT_BODY_Z INIT_BODY_YAW_DEG
      ;;
    *)
      echo "[localization] ERROR: mode must be mapping or global" >&2
      usage >&2
      exit 2
      ;;
  esac
else
  MODE="${FAST_LIO_MODE:-mapping}"
fi

case "${MODE}" in
  mapping|global) ;;
  *)
    echo "[localization] ERROR: FAST_LIO_MODE must be mapping or global (got ${MODE})" >&2
    exit 2
    ;;
esac
export FAST_LIO_MODE="${MODE}"

if [[ "${MODE}" == "global" ]]; then
  MAP_VALUE="${MAP_VALUE:-${GLOBAL_MAP_PCD:-}}"
  if [[ -z "${MAP_VALUE}" ]]; then
    echo "[localization] ERROR: global mode requires a PCD filename" >&2
    usage >&2
    exit 3
  fi
  [[ "${MAP_VALUE}" == *.pcd ]] || MAP_VALUE="${MAP_VALUE}.pcd"
  if [[ "${MAP_VALUE}" = /* ]]; then
    MAP_PATH="${MAP_VALUE}"
  elif [[ "${MAP_VALUE}" == */* ]]; then
    echo "[localization] ERROR: relative map must be a filename in ${WS_DIR}: ${MAP_VALUE}" >&2
    exit 3
  elif [[ -r "${WS_DIR}/${MAP_VALUE}" ]]; then
    MAP_PATH="${WS_DIR}/${MAP_VALUE}"
  elif [[ -r "${WS_DIR}/src/localization/FAST_LIO/PCD/${MAP_VALUE}" ]]; then
    MAP_PATH="${WS_DIR}/src/localization/FAST_LIO/PCD/${MAP_VALUE}"
    echo "[localization] WARN: using legacy package PCD path; prefer ${WS_DIR}/${MAP_VALUE}" >&2
  else
    MAP_PATH="${WS_DIR}/${MAP_VALUE}"
  fi
  if [[ ! -r "${MAP_PATH}" ]]; then
    echo "[localization] ERROR: shared PCD is not readable: ${MAP_PATH}" >&2
    exit 3
  fi
  MAP_SHA256="$(sha256sum "${MAP_PATH}" | awk '{print $1}')"
  if [[ "${CLI_SELECTED}" == false && -n "${GLOBAL_MAP_SHA256:-}" &&
        "${MAP_SHA256}" != "${GLOBAL_MAP_SHA256}" ]]; then
    echo "[localization] ERROR: shared PCD hash mismatch" >&2
    echo "[localization] expected=${GLOBAL_MAP_SHA256} actual=${MAP_SHA256} file=${MAP_PATH}" >&2
    exit 5
  fi
  export GLOBAL_MAP_PCD="${MAP_PATH}"
  export GLOBAL_MAP_SHA256="${MAP_SHA256}"
  for value in "${INIT_BODY_X:-0}" "${INIT_BODY_Y:-0}" "${INIT_BODY_Z:-0}" "${INIT_BODY_YAW_DEG:-0}"; do
    is_number "${value}" || {
      echo "[localization] ERROR: configured initial pose values must be numbers" >&2
      exit 2
    }
  done
  echo "[localization] mode=global map=${MAP_PATH} sha256=${MAP_SHA256}"
  echo "[localization] initial body pose=(${INIT_BODY_X:-0}, ${INIT_BODY_Y:-0}, ${INIT_BODY_Z:-0}) yaw=${INIT_BODY_YAW_DEG:-0}deg"
else
  echo "[localization] mode=mapping (local startup origin)"
fi

LIDAR_PID=""
LIO_PID=""
EKF_PID=""
cleanup() {
  trap - TERM INT EXIT
  [[ -z "${EKF_PID}" ]] || kill "${EKF_PID}" 2>/dev/null || true
  [[ -z "${LIO_PID}" ]] || kill "${LIO_PID}" 2>/dev/null || true
  [[ -z "${LIDAR_PID}" ]] || kill "${LIDAR_PID}" 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup TERM INT EXIT

roslaunch fast_lio lidar.launch & LIDAR_PID=$!
sleep 2

if [[ "${MODE}" == "global" ]]; then
  roslaunch fast_lio global_localization_mid360.launch & LIO_PID=$!
else
  roslaunch fast_lio mapping_mid360.launch & LIO_PID=$!
fi

# Do not start the downstream EKF until FAST-LIO has produced a real odometry
# sample.  In global mode this also proves initial alignment completed.
if ! timeout "${FAST_LIO_ODOM_TIMEOUT:-90}" rostopic echo -n 1 /Odometry >/dev/null; then
  echo "[localization] ERROR: timed out waiting for /Odometry" >&2
  exit 6
fi

roslaunch ekf_quat ekf_quat_lidar_mavros.launch & EKF_PID=$!

set +e
wait -n "${LIDAR_PID}" "${LIO_PID}" "${EKF_PID}"
rc=$?
set -e
exit "${rc}"
