#!/usr/bin/env bash
# No-hardware lifecycle tests for fly.sh and localization.sh.
# Run inside the ROS Noetic container from the dls_ws root:
#   bash tests/test_startup_scripts.sh
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM_LOG="$(mktemp)"
CASE_OUT="$(mktemp)"
export SIM_LOG

cleanup_test_files() {
  rm -f "${SIM_LOG}" "${CASE_OUT}"
}
trap cleanup_test_files EXIT

fail() {
  echo "FAIL: $*" >&2
  exit 1
}

assert_contains() {
  local pattern="$1"
  local file="${2:-${SIM_LOG}}"
  grep -Fq -- "${pattern}" "${file}" || fail "missing '${pattern}' in ${file}"
}

assert_count() {
  local expected="$1"
  local pattern="$2"
  local actual
  actual="$(grep -Fc -- "${pattern}" "${SIM_LOG}" || true)"
  [[ "${actual}" == "${expected}" ]] || fail "expected ${expected} x '${pattern}', got ${actual}"
}

assert_no_sim_children() {
  local pid
  while read -r pid; do
    [[ -n "${pid}" ]] || continue
    if kill -0 "${pid}" 2>/dev/null; then
      fail "simulated roslaunch child still alive: pid=${pid}"
    fi
  done < <(awk '$1 == "START" {print $2}' "${SIM_LOG}" | sort -u)
}

reset_case() {
  : > "${SIM_LOG}"
  : > "${CASE_OUT}"
  unset SIM_SERVICE_UNAVAILABLE SIM_MAVROS_EARLY_EXIT SIM_MAVCMD_FAIL
  unset SIM_LIDAR_EARLY_EXIT SIM_LIO_EARLY_EXIT SIM_EKF_EARLY_EXIT SIM_ODOM_TIMEOUT
}

roslaunch() {
  printf 'START %s roslaunch %s\n' "${BASHPID}" "$*" >> "${SIM_LOG}"
  if [[ "${1:-}" == "mavros" && -n "${SIM_MAVROS_EARLY_EXIT:-}" ]]; then
    /bin/sleep 0.05
    return 0
  fi
  if [[ "$*" == "fast_lio lidar.launch" && -n "${SIM_LIDAR_EARLY_EXIT:-}" ]]; then
    return 11
  fi
  if [[ "$*" == "fast_lio mapping_mid360.launch" ||
        "$*" == "fast_lio global_localization_mid360.launch" ]]; then
    if [[ -n "${SIM_LIO_EARLY_EXIT:-}" ]]; then
      return 12
    fi
  fi
  if [[ "$*" == "ekf_quat ekf_quat_lidar_mavros.launch" && -n "${SIM_EKF_EARLY_EXIT:-}" ]]; then
    return 13
  fi
  trap 'exit 0' TERM INT
  while true; do /bin/sleep 1; done
}

rosservice() {
  printf 'CALL %s rosservice %s\n' "${BASHPID}" "$*" >> "${SIM_LOG}"
  [[ -z "${SIM_SERVICE_UNAVAILABLE:-}" ]]
}

rosrun() {
  printf 'CALL %s rosrun %s\n' "${BASHPID}" "$*" >> "${SIM_LOG}"
  return 0
}

rostopic() {
  printf 'CALL %s rostopic %s\n' "${BASHPID}" "$*" >> "${SIM_LOG}"
  return 0
}

# Speed up fixed startup sleeps without changing the long-lived fake nodes.
sleep() {
  /bin/sleep 0.01
}

# Preserve the scripts' command shape while making timeout outcomes injectable.
timeout() {
  local duration="$1"
  shift
  if [[ "${1:-}" == "rosrun" && -n "${SIM_MAVCMD_FAIL:-}" ]]; then
    printf 'TIMEOUT %s %s\n' "${duration}" "$*" >> "${SIM_LOG}"
    return 124
  fi
  if [[ "${1:-}" == "rostopic" && -n "${SIM_ODOM_TIMEOUT:-}" ]]; then
    printf 'TIMEOUT %s %s\n' "${duration}" "$*" >> "${SIM_LOG}"
    return 124
  fi
  "$@"
}

export -f roslaunch rosservice rosrun rostopic sleep timeout

run_expect() {
  local expected="$1"
  shift
  set +e
  "$@" >"${CASE_OUT}" 2>&1
  local rc=$?
  set -e
  [[ "${rc}" == "${expected}" ]] || {
    tail -n 40 "${CASE_OUT}" >&2
    fail "expected rc=${expected}, got rc=${rc}: $*"
  }
}

cd "${WS_DIR}"

# CLI validation must fail before any ROS process is started.
reset_case
run_expect 2 bash ./fly.sh invalid
run_expect 3 bash ./fly.sh global missing-map.pcd
run_expect 2 bash ./fly.sh global origin.pcd x 0 0 0
[[ ! -s "${SIM_LOG}" ]] || fail "CLI validation unexpectedly started ROS commands"

# Healthy mapping stack remains alive and launches every expected component.
reset_case
run_expect 124 /usr/bin/timeout 1 env MAVROS_SERVICE_TIMEOUT=2 MAVCMD_TIMEOUT=2 bash ./fly.sh mapping
assert_contains "roslaunch mavros px4.launch"
assert_count 5 "rosrun mavros mavcmd long 511"
assert_contains "roslaunch fast_lio lidar.launch"
assert_contains "roslaunch fast_lio mapping_mid360.launch"
assert_contains "roslaunch ekf_quat ekf_quat_lidar_mavros.launch"
assert_no_sim_children

# Healthy global stack resolves the legacy fixture map and selects global LIO.
reset_case
run_expect 124 /usr/bin/timeout 1 env MAVROS_SERVICE_TIMEOUT=2 MAVCMD_TIMEOUT=2 \
  bash ./fly.sh global origin.pcd 1 2 3 45
assert_contains "roslaunch fast_lio global_localization_mid360.launch"
assert_contains "initial body pose=(1, 2, 3) yaw=45deg" "${CASE_OUT}"
assert_no_sim_children

# MAVROS service timeout is a hard startup failure and cleans MAVROS.
reset_case
export SIM_SERVICE_UNAVAILABLE=1
run_expect 4 env MAVROS_SERVICE_TIMEOUT=1 MAVCMD_TIMEOUT=2 bash ./fly.sh mapping
assert_contains "unavailable after 1s" "${CASE_OUT}"
assert_no_sim_children

# A clean roslaunch exit is still an unexpected MAVROS failure (rc must be nonzero).
reset_case
export SIM_SERVICE_UNAVAILABLE=1 SIM_MAVROS_EARLY_EXIT=1
run_expect 1 env MAVROS_SERVICE_TIMEOUT=2 MAVCMD_TIMEOUT=2 bash ./fly.sh mapping
assert_contains "MAVROS exited before" "${CASE_OUT}"
assert_no_sim_children

# Individual mavcmd failures are warnings; localization must still start.
reset_case
export SIM_MAVCMD_FAIL=1
run_expect 124 /usr/bin/timeout 1 env MAVROS_SERVICE_TIMEOUT=2 MAVCMD_TIMEOUT=2 bash ./fly.sh mapping
assert_count 5 "TIMEOUT 2 rosrun mavros mavcmd"
assert_contains "roslaunch fast_lio mapping_mid360.launch"
assert_no_sim_children

# LIO early exit propagates through localization and terminates MAVROS.
reset_case
export SIM_LIO_EARLY_EXIT=1
run_expect 12 env MAVROS_SERVICE_TIMEOUT=2 MAVCMD_TIMEOUT=2 bash ./fly.sh mapping
assert_contains "localization exited unexpectedly (rc=12)" "${CASE_OUT}"
assert_no_sim_children

# Missing odometry fails localization and cleans both LiDAR and LIO children.
reset_case
export SIM_ODOM_TIMEOUT=1
run_expect 6 env FAST_LIO_ODOM_TIMEOUT=1 bash ./localization.sh mapping
assert_contains "timed out waiting for /Odometry" "${CASE_OUT}"
assert_no_sim_children

# Environment-driven global mode rejects a fleet map hash mismatch before launch.
reset_case
run_expect 5 env FAST_LIO_MODE=global GLOBAL_MAP_PCD=origin.pcd GLOBAL_MAP_SHA256=bad \
  bash ./localization.sh
assert_contains "shared PCD hash mismatch" "${CASE_OUT}"
[[ ! -s "${SIM_LOG}" ]] || fail "hash mismatch unexpectedly started ROS commands"

echo "ALL-STARTUP-SCRIPT-TESTS-PASS"
