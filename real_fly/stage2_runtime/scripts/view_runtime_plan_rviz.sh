#!/usr/bin/env bash
# Open a local NX RViz preview. Publishes visualization topics only.
set -euo pipefail

if [[ $# -ne 2 ]]; then
  echo "Usage: $0 RUN_ID TASK_ID" >&2
  exit 2
fi
run_id="$1"
task_id="$2"
[[ "$run_id" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] || { echo "Invalid RUN_ID" >&2; exit 2; }
[[ "$task_id" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] || { echo "Invalid TASK_ID" >&2; exit 2; }

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
runtime_root="$(cd -- "$script_dir/.." && pwd)"
package="$runtime_root/map_packages/$run_id"
task="$runtime_root/missions/$run_id/$task_id"
map="/home/nv/dls_ws/${run_id}.pcd"
scene="$package/approved_scene_graph.json"
mission="$task/planning/mission_plan.json"
candidates="$task/planning/candidates.json"
rviz_config="$runtime_root/rviz/runtime_plan.rviz"
for path in "$map" "$scene" "$mission" "$candidates" "$rviz_config"; do
  [[ -s "$path" ]] || { echo "Missing preview input: $path" >&2; exit 2; }
done
[[ -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ]] || {
  echo "No graphical display. Run this command in the NX desktop terminal, not plain SSH." >&2
  exit 2
}

unset _CATKIN_SETUP_DIR || true
set +u
source /opt/ros/noetic/setup.bash
source /home/nv/dls_ws/devel/setup.bash
set -u
rosnode list >/dev/null 2>&1 || {
  echo "ROS master unavailable; keep global localization running first." >&2
  exit 2
}

map_pid=""
preview_pid=""
cleanup() {
  trap - EXIT INT TERM
  [[ -z "$preview_pid" ]] || kill "$preview_pid" 2>/dev/null || true
  [[ -z "$map_pid" ]] || kill "$map_pid" 2>/dev/null || true
  [[ -z "$preview_pid" ]] || wait "$preview_pid" 2>/dev/null || true
  [[ -z "$map_pid" ]] || wait "$map_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "Safety scope: RViz visualization only; no planner, controller, goal, arm, or takeoff."
rosrun pcl_ros pcd_to_pointcloud "$map" 0.0 \
  _frame_id:=world _latch:=true cloud_pcd:=/pre_map_vln/preview/map &
map_pid=$!
python3 "$script_dir/publish_runtime_plan_preview.py" \
  --scene "$scene" --mission "$mission" --candidates "$candidates" &
preview_pid=$!
sleep 2
rviz -d "$rviz_config"
