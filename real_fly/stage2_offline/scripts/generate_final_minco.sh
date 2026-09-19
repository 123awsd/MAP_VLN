#!/usr/bin/env bash
# Host-only, isolated ROS master; no hardware or command consumers.
set -euo pipefail
[[ $# -eq 2 ]] || { echo "Usage: $0 RUN_ID TASK_ID" >&2; exit 2; }
run=$1 task=$2
[[ "$run" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ && "$task" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] || exit 2
root="$(cd "$(dirname "$0")/../../.." && pwd)"
cd "$root"
mission="real_fly/stage2_offline/data/$run/tasks/$task/planning/mission_plan.json"
map="real_fly/stage2_offline/data/$run/fastlio_complete/handheld_map_${run}_complete.pcd"
dest="real_fly/stage2_runtime/missions/$run/$task"
[[ -s "$mission" && -s "$map" ]] || { echo "Missing mission/map" >&2; exit 2; }
[[ ! -e "$dest/final_minco_manifest.json" ]] || { echo "Final MINCO already exists; use a new task or archive it first." >&2; exit 2; }
bash real_fly/stage2_offline/scripts/build_host_minco.sh
mkdir -p "$dest"
job=$(mktemp -d "$dest/.minco-build-XXXXXX")
trap 'rc=$?; if ((rc)); then echo "Final MINCO failed; diagnostics preserved: $root/$job/generation.log" >&2; fi' EXIT
# Only publish the complete set after MINCO generation and validation succeed.
cp "$mission" "$job/source_mission.json"
if [[ ! -s "$dest/execution_bundle.json" ]]; then
  bash real_fly/stage2_runtime/scripts/prepare_real_execution.sh "$run" "$root/$mission" "$task"
fi
cp "$dest/execution_bundle.json" "$job/execution_bundle.json"
cp real_fly/stage2_runtime/config/super_indoor_stage2.yaml "$job/planner.yaml"
docker run --rm --network none --user "$(id -u):$(id -g)" \
  -v "$root:/workspace/project" -e JOB="/workspace/project/$job" \
  -e MAP="/workspace/project/$map" pre-map-vln/real-minco:local bash -c '
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /workspace/project/real_fly/stage2_offline/runtime/minco_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
export ROS_HOME="$JOB/ros"
python3 /workspace/project/real_fly/stage2_offline/scripts/export_full_smooth_route.py --mission "$JOB/source_mission.json" --output "$JOB/full_smooth_route.txt" --path-source astar
python3 /workspace/project/real_fly/stage2_runtime/scripts/build_super_collision_pcd.py "$MAP" "$JOB/collision.pcd" --voxel 0.10 --min-points-per-voxel 100
roscore > "$JOB/roscore.log" 2>&1 & master=$!
trap "kill $master 2>/dev/null || true" EXIT
for i in $(seq 1 50); do rosparam list >/dev/null 2>&1 && break; sleep .1; done
rosparam list >/dev/null
timeout 900 rosrun mission_planner full_smooth_mission \
  _preview_only:=true _preview_start_at_first_route_point:=true _exit_after_preview:=true \
  _start_trigger_type:=3 _save_generated_trajectory:=true \
  _saved_trajectory_path:="$JOB/final_minco.txt" _route_path:="$JOB/full_smooth_route.txt" \
  _known_map_pcd:="$JOB/collision.pcd" _super_config_path:="$JOB/planner.yaml" \
  _use_super_safe_corridor:=true _super_use_recorded_guide_without_astar:=true \
  _super_static_map_only:=true _local_collision_replan_enabled:=false \
  _collision_clearance:=0.25 _auto_land:=false \
  _max_acceleration:=6.0 _max_jerk:=90.0 _max_snap:=350.0 \
  _max_yaw_rate:=1.5 _max_yaw_lock_variation:=1.0 _max_jerk_discontinuity:=0.0001 \
  _yaw_velocity_threshold:=0.08 _yaw_lookahead_time:=0.40 \
  _yaw_start_blend_duration:=1.5 _yaw_terminal_blend_duration:=2.0 \
  > "$JOB/generation.log" 2>&1
test -s "$JOB/final_minco.txt"
python3 /workspace/project/real_fly/stage2_runtime/scripts/saved_minco_artifact.py seal \
  --directory "$JOB" --map "$MAP" --mission "$JOB/source_mission.json" --clearance 0.25
'
for name in final_minco.txt collision.pcd planner.yaml full_smooth_route.txt final_minco_preview.json; do
  cp "$job/$name" "$dest/$name"
done
cp "$job/final_minco_manifest.json" "$dest/final_minco_manifest.json"
echo "Final MINCO ready: $root/$dest/final_minco.txt"
echo "Generation log: $root/$job/generation.log"
