#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STAGE1_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PROFILE="${REALSENSE_RECORDING_PROFILE:-$STAGE1_ROOT/config/realsense_d435_recording.conf}"
RGB_NODE="${REALSENSE_RGB_DYNAMIC_NODE:-/camera/rgb_camera}"

[[ -r "$PROFILE" ]] || { echo "RealSense recording profile is missing: $PROFILE" >&2; exit 1; }
# shellcheck disable=SC1090
source "$PROFILE"

[[ "${REALSENSE_RGB_AUTO_EXPOSURE:-}" == "true" || "${REALSENSE_RGB_AUTO_EXPOSURE:-}" == "false" ]] || {
  echo "REALSENSE_RGB_AUTO_EXPOSURE must be true or false in $PROFILE" >&2
  exit 2
}
if [[ "$REALSENSE_RGB_AUTO_EXPOSURE" == "false" && ! "${REALSENSE_RGB_EXPOSURE:-}" =~ ^[0-9]+$ ]]; then
  echo "Invalid RGB exposure in $PROFILE" >&2
  exit 2
fi
command -v rosrun >/dev/null 2>&1 || { echo "rosrun not found; source ROS Noetic first." >&2; exit 1; }

deadline=$((SECONDS + 20))
until rosrun dynamic_reconfigure dynparam list 2>/dev/null | grep -Fxq "$RGB_NODE"; do
  (( SECONDS < deadline )) || {
    echo "Timed out waiting for RealSense dynamic node: $RGB_NODE" >&2
    exit 1
  }
  sleep 1
done

rosrun dynamic_reconfigure dynparam set "$RGB_NODE" enable_auto_exposure "$REALSENSE_RGB_AUTO_EXPOSURE" >/dev/null
if [[ "$REALSENSE_RGB_AUTO_EXPOSURE" == "false" ]]; then
  rosrun dynamic_reconfigure dynparam set "$RGB_NODE" exposure "$REALSENSE_RGB_EXPOSURE" >/dev/null
fi

actual="$(rosrun dynamic_reconfigure dynparam get "$RGB_NODE")"
expected_auto="False"
[[ "$REALSENSE_RGB_AUTO_EXPOSURE" == "true" ]] && expected_auto="True"
grep -Eq "enable_auto_exposure['\"]?: (${REALSENSE_RGB_AUTO_EXPOSURE}|${expected_auto})" <<<"$actual" || {
  echo "RealSense rejected RGB automatic-exposure mode=$REALSENSE_RGB_AUTO_EXPOSURE." >&2
  exit 1
}
if [[ "$REALSENSE_RGB_AUTO_EXPOSURE" == "false" ]]; then
  grep -Eq "exposure['\"]?: ${REALSENSE_RGB_EXPOSURE}(\.0)?([,}]|$)" <<<"$actual" || {
    echo "RealSense rejected RGB exposure ${REALSENSE_RGB_EXPOSURE}." >&2
    exit 1
  }
  echo "OK: D435 RGB manual exposure=${REALSENSE_RGB_EXPOSURE} ($RGB_NODE)"
else
  echo "OK: D435 RGB automatic exposure enabled ($RGB_NODE)"
fi
