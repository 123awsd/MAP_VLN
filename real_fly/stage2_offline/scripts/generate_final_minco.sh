#!/usr/bin/env bash
# Host-only, isolated ROS master; no hardware or command consumers.
set -euo pipefail
[[ $# -ge 2 && $# -le 6 ]] || { echo "Usage: $0 RUN_ID TASK_ID [CLEARANCE_M] [PATH_SOURCE] [SPEED_MPS] [MAX_YAW_RATE_RAD_S]" >&2; exit 2; }
run=$1 task=$2 clearance="${3:-0.25}" path_source="${4:-clearance_optimized}"
speed="${5:-0.6}" max_yaw_rate="${6:-1.5}"
departure_validation=false
[[ "$run" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ && "$task" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] || exit 2
case "$path_source" in
  validated|clearance_optimized|astar|sparse|sparse_astar) ;;
  *) echo "Invalid path source: $path_source" >&2; exit 2 ;;
esac
[[ "$clearance" =~ ^0[.][0-9]+$|^[1-9][0-9]*([.][0-9]+)?$ ]] || { echo "Invalid clearance: $clearance" >&2; exit 2; }
python3 - "$clearance" <<'PY'
import sys
value = float(sys.argv[1])
if not 0.10 <= value <= 1.00:
    raise SystemExit(f"Clearance must be in [0.10, 1.00] m, got {value}")
PY
python3 - "$speed" "$max_yaw_rate" <<'PY'
import math
import sys

speed, yaw_rate = map(float, sys.argv[1:])
if not math.isfinite(speed) or not 0.1 <= speed <= 2.0:
    raise SystemExit(f"Speed must be in [0.1, 2.0] m/s, got {speed}")
if not math.isfinite(yaw_rate) or not 0.1 <= yaw_rate <= 3.0:
    raise SystemExit(f"Maximum yaw rate must be in [0.1, 3.0] rad/s, got {yaw_rate}")
PY
root="$(cd "$(dirname "$0")/../../.." && pwd)"
cd "$root"
mission="real_fly/stage2_offline/data/$run/tasks/$task/planning/mission_plan.json"
if [[ -s "real_fly/stage2_offline/data/$run/tasks/$task/departure_guide_report.json" ]]; then
  departure_validation=true
fi
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
python3 - "$job/planner.yaml" "$clearance" "$speed" <<'PY'
import pathlib
import sys
import yaml

path = pathlib.Path(sys.argv[1])
document = yaml.safe_load(path.read_text(encoding="utf-8"))
document["super_planner"]["robot_r"] = float(sys.argv[2])
document["traj_opt"]["boundary"]["max_vel"] = float(sys.argv[3])
path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
PY
sudo docker run --rm --network none --user "$(id -u):$(id -g)" \
  -v "$root:/workspace/project" -e JOB="/workspace/project/$job" \
  -e MAP="/workspace/project/$map" -e COLLISION_CLEARANCE="$clearance" \
  -e MINCO_PATH_SOURCE="$path_source" -e MINCO_SPEED_MPS="$speed" \
  -e MINCO_MAX_YAW_RATE="$max_yaw_rate" \
  -e DEPARTURE_VALIDATION="$departure_validation" \
  pre-map-vln/real-minco:local bash -c '
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /workspace/project/real_fly/stage2_offline/runtime/minco_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
export ROS_HOME="$JOB/ros"
python3 /workspace/project/real_fly/stage2_offline/scripts/export_full_smooth_route.py --mission "$JOB/source_mission.json" --output "$JOB/full_smooth_route.txt" --path-source "$MINCO_PATH_SOURCE" --speed "$MINCO_SPEED_MPS"
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
  _collision_clearance:="$COLLISION_CLEARANCE" _auto_land:=false \
  _max_acceleration:=6.0 _max_jerk:=90.0 _max_snap:=350.0 \
  _max_yaw_rate:="$MINCO_MAX_YAW_RATE" _max_yaw_lock_variation:=1.0 _max_jerk_discontinuity:=0.0001 \
  _narrow_corridor/enabled:=true _narrow_corridor/clearance_threshold:=0.40 \
  _narrow_corridor/max_width:=1.00 _narrow_corridor/transition_length:=0.45 \
  _narrow_corridor/max_half_width:=0.18 _narrow_corridor/min_half_width:=0.10 \
  _narrow_corridor/margin:=0.03 _narrow_corridor/side_vertical_window:=0.35 \
  _narrow_corridor/side_longitudinal_window:=0.35 _narrow_corridor/max_plane_violation:=0.005 \
  _vertical_guide_floor/enabled:=true _vertical_guide_floor/max_violation:=0.002 \
  _guide_tracking/enabled:=true _guide_tracking/horizontal_half_width:=0.05 \
  _guide_tracking/monotonic_vertical_band:=0.01 \
  _guide_tracking/max_plane_violation:=0.005 \
  _yaw_velocity_threshold:=0.08 _yaw_lookahead_time:=0.40 \
  _yaw_start_blend_duration:=1.5 _yaw_terminal_blend_duration:=2.0 \
  > "$JOB/generation.log" 2>&1
test -s "$JOB/final_minco.txt"
test -s "$JOB/narrow_corridor_report.json"
if [[ "$DEPARTURE_VALIDATION" == true ]]; then
  python3 /workspace/project/real_fly/stage2_offline/scripts/verify_departure_geometry.py \
    --artifact "$JOB/final_minco.txt" --map "$MAP" \
    --output "$JOB/departure_geometry_report.json"
fi
python3 /workspace/project/real_fly/stage2_runtime/scripts/saved_minco_artifact.py seal \
  --directory "$JOB" --map "$MAP" --mission "$JOB/source_mission.json" --clearance "$COLLISION_CLEARANCE"
'
for name in final_minco.txt collision.pcd planner.yaml full_smooth_route.txt final_minco_preview.json narrow_corridor_report.json; do
  cp "$job/$name" "$dest/$name"
done
if [[ -s "$job/departure_geometry_report.json" ]]; then
  cp "$job/departure_geometry_report.json" "$dest/departure_geometry_report.json"
fi
cp "$job/final_minco_manifest.json" "$dest/final_minco_manifest.json"
echo "Final MINCO ready: $root/$dest/final_minco.txt"
echo "Generation log: $root/$job/generation.log"
