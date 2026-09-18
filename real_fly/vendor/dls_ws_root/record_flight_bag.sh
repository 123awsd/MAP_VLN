#!/usr/bin/env bash
# Record one real-flight session with the inputs and outputs needed to analyse
# localization, SUPER/full-smooth planning, PX4Ctrl tracking, and PX4 state.
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

PROFILE="analysis"
EXTRA_TOPICS=()

usage() {
  cat <<'EOF'
Usage: ./record_flight_bag.sh [--profile analysis|replay] [--extra-topic TOPIC]...

Profiles:
  analysis (default)  Record the complete localization -> planning -> PX4Ctrl
                      chain, including the registered planning cloud and SUPER
                      replanning products. Suitable for normal flight tests.
  replay              Add raw Livox LiDAR and IMU so FAST-LIO can be replayed.
                      This has a substantially higher disk/write-rate cost.

The default profile includes the localization -> planning -> control chain.
Use --extra-topic for any additional external mission sensor.
EOF
}

while (($# > 0)); do
  case "$1" in
    --profile)
      (($# >= 2)) || { echo "--profile requires a value" >&2; exit 2; }
      PROFILE="$2"
      shift 2
      ;;
    --extra-topic)
      (($# >= 2)) || { echo "--extra-topic requires a topic name" >&2; exit 2; }
      EXTRA_TOPICS+=("$2")
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "${PROFILE}" in
  analysis|replay) ;;
  *)
    echo "Unsupported profile: ${PROFILE}" >&2
    usage >&2
    exit 2
    ;;
esac

# Parse all script arguments before sourcing catkin. catkin's setup.bash passes
# the caller's positional arguments to its setup helper, so sourcing it earlier
# would incorrectly treat --profile/--help as catkin arguments.
source /opt/ros/noetic/setup.bash
source "${WS_DIR}/devel/setup.bash"

BAG_ROOT="${WS_DIR}/flight_bags"
SESSION_NAME="flight_$(date +%Y%m%d_%H%M%S)"
SESSION_DIR="${BAG_ROOT}/${SESSION_NAME}"
CONTEXT_DIR="${SESSION_DIR}/context"
mkdir -p "${CONTEXT_DIR}"

snapshot_file() {
  local source="$1"
  local destination="$2"
  if [[ -f "${source}" ]]; then
    cp -a "${source}" "${destination}"
  fi
}

snapshot_param() {
  local param_name="$1"
  local destination="$2"
  if ! rosparam get "${param_name}" > "${destination}" 2>&1; then
    printf '# Parameter unavailable when recording started: %s\n' "${param_name}" > "${destination}"
  fi
}

{
  printf 'session: %s\n' "${SESSION_NAME}"
  printf 'profile: %s\n' "${PROFILE}"
  printf 'started_at: %s\n' "$(date --iso-8601=seconds)"
  printf 'git_commit: %s\n' "$(git -C "${WS_DIR}" rev-parse HEAD)"
  printf 'git_status: |\n'
  git -C "${WS_DIR}" status --short | sed 's/^/  /'
  printf 'extra_topics:\n'
  for topic in "${EXTRA_TOPICS[@]}"; do
    printf '  - %s\n' "${topic}"
  done
} > "${SESSION_DIR}/session.yaml"

# The exact static inputs are necessary for repeatable full-smooth clearance
# checks. Copy only files that exist; the script also works for SUPER-only runs.
for artifact in \
  recorded_environment_map.pcd \
  recorded_full_smooth_route.txt \
  recorded_full_smooth_trajectory.txt \
  recorded_waypoints.txt; do
  snapshot_file "${WS_DIR}/${artifact}" "${CONTEXT_DIR}/${artifact}"
done

snapshot_file "${WS_DIR}/src/SUPER/mission_planner/launch/competition_2026_planner.launch" \
              "${CONTEXT_DIR}/competition_2026_planner.launch"
snapshot_file "${WS_DIR}/src/SUPER/super_planner/config/competition_2026_real.yaml" \
              "${CONTEXT_DIR}/competition_2026_real.yaml"
snapshot_file "${WS_DIR}/src/SUPER/mission_planner/config/competition_2026_cloud_guard_real.yaml" \
              "${CONTEXT_DIR}/competition_2026_cloud_guard_real.yaml"
snapshot_file "${WS_DIR}/src/SUPER/control/px4ctrl/config/ctrl_param_competition_2026.yaml" \
              "${CONTEXT_DIR}/ctrl_param_competition_2026.yaml"

# Parameters capture launch-time overrides that are not represented by the
# checked-in YAML files. An unavailable optional node is recorded explicitly.
snapshot_param /px4ctrl "${CONTEXT_DIR}/px4ctrl_params.yaml"
snapshot_param /full_smooth_mission "${CONTEXT_DIR}/full_smooth_mission_params.yaml"
snapshot_param /fsm_node "${CONTEXT_DIR}/fsm_node_params.yaml"
snapshot_param /competition_cloud_guard "${CONTEXT_DIR}/competition_cloud_guard_params.yaml"
snapshot_param /competition_command_mux "${CONTEXT_DIR}/competition_command_mux_params.yaml"
snapshot_param /competition_target_detector "${CONTEXT_DIR}/competition_target_detector_params.yaml"

rosnode list > "${SESSION_DIR}/rosnode_list.txt" 2>&1 || true
rostopic list -v > "${SESSION_DIR}/rostopic_list.txt" 2>&1 || true

# Localisation and planner inputs. /competition/planning_cloud is the exact
# live cloud consumed by ROG-Map; the original /cloud_registered is retained to
# diagnose cloud-guard frame/latency issues without recording raw LiDAR by
# default.
TOPICS=(
  /ekf_quat/ekf_odom
  /Odometry
  /DebugOdometry
  /LioDebug
  /cloud_registered
  /competition/planning_cloud
  /competition/static_constraint_cloud
  /tf
  /tf_static

  # Planner, command-mux, and full-smooth handoff.
  /planning/click_goal
  /planning/waypoint_path
  /planning/pos_cmd
  /planning/super_pos_cmd
  /planning/gate_pos_cmd
  /planning/gate_active
  /planning/super_pause
  /planning/d_gate_smooth_path
  /planning/d_gate_smooth_markers
  /planning_cmd/poly_traj
  /traj_start_trigger
  /px4ctrl/takeoff_land

  # SUPER replanning evidence. Marker topics are sparse and preserve the A*,
  # corridor, committed/backup trajectory, and collision diagnostics.
  /fsm_node/fsm/path
  /fsm_node/visualization/frontend_path
  /fsm_node/visualization/exp_traj
  /fsm_node/visualization/backup_traj
  /fsm_node/visualization/committed_traj
  /fsm_node/visualization/exp_sfc
  /fsm_node/visualization/backup_sfc
  /fsm_node/visualization/astar_debug
  /fsm_node/visualization/ciri_debug_mkr
  /fsm_node/visualization/ciri_debug_pc
  /fsm_node/visualization/replan_log_mkr
  /fsm_node/visualization/replan_log_pc

  # PX4Ctrl inputs, control output, and vehicle/RC state.
  /debugPx4ctrl
  /mavros/state
  /mavros/extended_state
  /mavros/rc/in
  /mavros/imu/data
  /mavros/imu/data_raw
  /mavros/battery
  /mavros/local_position/odom
  /mavros/local_position/pose
  /mavros/local_position/velocity_local
  /mavros/setpoint_raw/attitude
  /diagnostics
  /rosout
  /rosout_agg
)

if [[ "${PROFILE}" == "replay" ]]; then
  TOPICS+=(
    /livox/lidar
    /livox/imu
    /cloud_registered_body
    /cloud_effected
  )
fi

TOPICS+=("${EXTRA_TOPICS[@]}")

echo "Recording ${PROFILE} session in ${SESSION_DIR}"
echo "Press Ctrl+C only after landing; rosbag will close all split files."
echo "Recorded topics: ${#TOPICS[@]}"

# Splitting protects the recorder from large individual bag files. LZ4 keeps
# CPU use low; the larger buffer absorbs point-cloud bursts without changing
# planner/controller execution.
rosbag record --lz4 --tcpnodelay --split --size=2048 --buffsize=512 --chunksize=768 \
  -O "${SESSION_DIR}/${SESSION_NAME}" \
  "${TOPICS[@]}"
