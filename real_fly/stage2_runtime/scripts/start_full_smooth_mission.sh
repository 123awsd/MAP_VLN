#!/usr/bin/env bash
# Start direct full_smooth_mission execution for an approved offline route.
# This starts no PX4Ctrl, does not arm/take off, and never requests landing.
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  start_full_smooth_mission.sh /absolute/path/to/map.pcd \
    --bundle /absolute/path/to/execution_bundle.json

The route is executed as a continuous MINCO command stream. Do not start the
old /fsm_node SUPER planner alongside it.
EOF
}

[[ $# -ge 3 ]] || { usage >&2; exit 2; }
map_pcd="$1"
shift
bundle=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --route|--clearance) echo "Route replanning/clearance overrides are disabled. Use the certified saved MINCO bundle." >&2; exit 2 ;;
    --bundle) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; bundle="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ "$map_pcd" = /* && -s "$map_pcd" ]] || {
  echo "Missing absolute map PCD: $map_pcd" >&2; exit 2;
}
[[ -s "$bundle" ]] || { echo "Missing execution bundle" >&2; exit 2; }
mission_dir="$(cd "$(dirname "$bundle")" && pwd)"
python3 "$(dirname "$0")/saved_minco_artifact.py" verify --directory "$mission_dir" --map "$map_pcd"
clearance="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["collision_clearance"])' "$mission_dir/final_minco_manifest.json")"

if [[ -n "$bundle" ]]; then
  [[ -s "$bundle" ]] || { echo "Missing execution bundle: $bundle" >&2; exit 2; }
  expected_sha="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["map_sha256"])' "$bundle")"
  actual_sha="$(sha256sum "$map_pcd" | awk '{print $1}')"
  [[ "$expected_sha" == "$actual_sha" ]] || {
    echo "execution bundle/map SHA256 mismatch" >&2; exit 2;
  }
  approved_start="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("safety",{}).get("start_pose_explicitly_approved",False))' "$bundle")"
  [[ "$approved_start" == "True" ]] || {
    echo "execution bundle start pose is not explicitly approved" >&2; exit 2;
  }
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
runtime_root="$(cd -- "$script_dir/.." && pwd)"
dls_ws="${DLS_WS:-/home/nv/dls_ws}"
launch="$runtime_root/launch/full_smooth_stage2.launch"

[[ -r "$launch" ]] || { echo "Missing launch file: $launch" >&2; exit 2; }
[[ -r "$dls_ws/devel/setup.bash" ]] || {
  echo "Missing DLS workspace build: $dls_ws/devel/setup.bash" >&2; exit 2;
}

unset _CATKIN_SETUP_DIR || true
set +u
source /opt/ros/noetic/setup.bash
source "$dls_ws/devel/setup.bash"
set -u

for node in /fsm_node /full_smooth_mission /competition_command_mux; do
  if rosnode list 2>/dev/null | grep -Fxq "$node"; then
    echo "${node} is already running; stop the previous planner terminal first." >&2
    exit 1
  fi
done
rostopic info /ekf_quat/ekf_odom >/dev/null 2>&1 || {
  echo "Missing /ekf_quat/ekf_odom; establish global localization first." >&2
  exit 1
}

echo "Starting direct full_smooth_mission (continuous MINCO; no /fsm_node)."
echo "Map: $map_pcd"
echo "Saved MINCO: $mission_dir/final_minco.txt (no replanning)"
echo "Clearance: ${clearance} m"
echo "Automatic landing: disabled"
echo "PX4Ctrl, takeoff, and flight trigger remain separate."

exec roslaunch "$launch" \
  "map_pcd:=$mission_dir/collision.pcd" \
  "trajectory_path:=$mission_dir/final_minco.txt" \
  "config_path:=$mission_dir/planner.yaml" \
  "collision_clearance:=$clearance"
