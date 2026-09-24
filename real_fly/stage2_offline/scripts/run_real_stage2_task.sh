#!/usr/bin/env bash
# Natural-language task -> 3-D mission -> collision-checked preview bundle.
# Offline only: final MINCO uses an isolated container ROS master, no hardware.
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  run_real_stage2_task.sh --run-id ID --task-id ID --instruction TEXT [options]
  run_real_stage2_task.sh --run-id ID --task-id ID --task-graph FILE [options]

Options:
  --start X Y Z YAW       Approved takeoff/hover pose in map frame. By default,
                          reuse planning_start.json for preview only.
  --budget-cny VALUE      Qwen parsing budget ceiling (default: 20).
  --max-candidates N      Maximum candidates retained per task (default: 6).
  --planning-config FILE  Override the per-run planning profile.
  --voxel-snapshot DIR    Override the matching voxel snapshot.
  --collision-clearance M CIRI/MINCO clearance; must match the planning profile
                          (default: 0.25).
  --speed-mps VALUE       Final route/MINCO speed limit (default: 0.6).
  --max-yaw-rate VALUE    Final yaw-rate limit in rad/s (default: 1.5).
  --static-demo           Freeze conditional tasks for a controlled fixed-layout demo.
  --visit-all-candidates  Exhaust all candidates of each task before the next task.
  --no-cache              Do not reuse the Qwen parser cache.
  -h, --help              Show this help.

Output:
  real_fly/stage2_offline/data/ID/tasks/TASK_ID/
EOF
}

run_id=""
task_id=""
instruction=""
task_graph_source=""
budget_cny="20"
max_candidates="6"
static_demo=0
visit_all_candidates=0
no_cache=0
planning_config_override=""
voxel_snapshot_override=""
collision_clearance="0.25"
speed_mps="0.6"
max_yaw_rate="1.5"
start_values=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --run-id) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; run_id="$2"; shift 2 ;;
    --task-id) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; task_id="$2"; shift 2 ;;
    --instruction) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; instruction="$2"; shift 2 ;;
    --task-graph) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; task_graph_source="$2"; shift 2 ;;
    --start)
      [[ $# -ge 5 ]] || { usage >&2; exit 2; }
      start_values=("$2" "$3" "$4" "$5")
      shift 5
      ;;
    --budget-cny) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; budget_cny="$2"; shift 2 ;;
    --max-candidates) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; max_candidates="$2"; shift 2 ;;
    --planning-config) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; planning_config_override="$2"; shift 2 ;;
    --voxel-snapshot) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; voxel_snapshot_override="$2"; shift 2 ;;
    --collision-clearance) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; collision_clearance="$2"; shift 2 ;;
    --speed-mps) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; speed_mps="$2"; shift 2 ;;
    --max-yaw-rate) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; max_yaw_rate="$2"; shift 2 ;;
    --static-demo) static_demo=1; shift ;;
    --visit-all-candidates) visit_all_candidates=1; shift ;;
    --no-cache) no_cache=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ "$run_id" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] || { echo "Invalid --run-id: $run_id" >&2; exit 2; }
[[ "$task_id" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] || { echo "Invalid --task-id: $task_id" >&2; exit 2; }
if [[ -n "$instruction" && -n "$task_graph_source" ]] || [[ -z "$instruction" && -z "$task_graph_source" ]]; then
  echo "Provide exactly one of --instruction or --task-graph." >&2
  exit 2
fi
[[ "$budget_cny" =~ ^[0-9]+([.][0-9]+)?$ ]] || { echo "Invalid --budget-cny" >&2; exit 2; }
[[ "$max_candidates" =~ ^[1-9][0-9]*$ ]] || { echo "Invalid --max-candidates" >&2; exit 2; }
[[ "$collision_clearance" =~ ^0[.][0-9]+$|^[1-9][0-9]*([.][0-9]+)?$ ]] || { echo "Invalid --collision-clearance" >&2; exit 2; }
python3 - "$speed_mps" "$max_yaw_rate" <<'PY'
import math
import sys

speed, yaw_rate = map(float, sys.argv[1:])
if not math.isfinite(speed) or not 0.1 <= speed <= 2.0:
    raise SystemExit(f"Speed must be in [0.1, 2.0] m/s, got {speed}")
if not math.isfinite(yaw_rate) or not 0.1 <= yaw_rate <= 3.0:
    raise SystemExit(f"Maximum yaw rate must be in [0.1, 3.0] rad/s, got {yaw_rate}")
PY

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
stage2_dir="$(cd -- "$script_dir/.." && pwd)"
root="$(cd -- "$stage2_dir/../.." && pwd)"
python="$root/.envs/habitat/bin/python"
run_data="$stage2_dir/data/$run_id"
scene_graph="$run_data/scene_graph.json"
if [[ -s "$run_data/approved_scene_graph.json" ]]; then
  scene_graph="$run_data/approved_scene_graph.json"
  echo "Using manually approved per-run room scene: $scene_graph"
else
  echo "No approved room scene found; using the original single-room scene: $scene_graph"
fi
voxel_snapshot="$run_data/voxel_snapshot"
planning_config="$stage2_dir/config/uav_3d_planning_real.yaml"
if [[ -n "$voxel_snapshot_override" ]]; then voxel_snapshot="$(realpath "$voxel_snapshot_override")"; fi
if [[ -n "$planning_config_override" ]]; then planning_config="$(realpath "$planning_config_override")"; fi
planning_start="$run_data/planning_start.json"
map_pcd="$run_data/fastlio_complete/handheld_map_${run_id}_complete.pcd"
task_dir="$run_data/tasks/$task_id"
task_graph="$task_dir/task_graph.json"
planning_dir="$task_dir/planning"
raw_mission="$planning_dir/mission_plan_astar.json"
mission="$planning_dir/mission_plan.json"
audit="$task_dir/mission_audit.json"
full_smooth_route="$planning_dir/full_smooth_route.txt"
runtime_bundle="$root/real_fly/stage2_runtime/missions/$run_id/$task_id/execution_bundle.json"

failed_backup_root="$run_data/tasks/.failed"
runtime_failed_backup_root="$root/real_fly/stage2_runtime/missions/.failed"

for path in "$python" "$scene_graph" "$voxel_snapshot/metadata.json" \
  "$voxel_snapshot/voxel_map.npz" "$planning_config" "$planning_start" "$map_pcd"; do
  [[ -s "$path" ]] || { echo "Missing Stage2 input: $path" >&2; exit 2; }
done

"$python" - "$planning_config" "$collision_clearance" <<'PY'
import math
import sys
import yaml

profile = yaml.safe_load(open(sys.argv[1], encoding="utf-8"))
configured = float(profile["planning"]["minimum_esdf_distance_m"])
requested = float(sys.argv[2])
if not math.isclose(configured, requested, rel_tol=0.0, abs_tol=1e-9):
    raise SystemExit(
        f"Planning/CIRI clearance mismatch: profile={configured:.3f} m, "
        f"--collision-clearance={requested:.3f} m"
    )
PY

# A failed parser/planner run leaves a partial task directory.  Preserve that
# diagnostic output, but allow the same TASK_ID to be retried.  A completed
# run still remains immutable and must use a new TASK_ID.
task_was_incomplete=0
if [[ -e "$task_dir" ]]; then
  if [[ -s "$task_dir/run_manifest.json" ]]; then
    echo "Completed task output exists; refusing to overwrite: $task_dir" >&2
    exit 2
  fi
  failed_stamp="$(date +%Y%m%d_%H%M%S)"
  failed_backup="$failed_backup_root/${task_id}.${failed_stamp}"
  mkdir -p "$failed_backup_root"
  mv "$task_dir" "$failed_backup"
  task_was_incomplete=1
  echo "Preserved incomplete task output: $failed_backup"
fi
if [[ -e "$runtime_bundle" ]]; then
  runtime_mission_dir="$(dirname "$runtime_bundle")"
  if [[ "$task_was_incomplete" -eq 0 ]]; then
    echo "Completed runtime mission exists; refusing to overwrite: $runtime_mission_dir" >&2
    exit 2
  fi
  failed_stamp="${failed_stamp:-$(date +%Y%m%d_%H%M%S)}"
  runtime_failed_backup="$runtime_failed_backup_root/$run_id/${task_id}.${failed_stamp}"
  mkdir -p "$runtime_failed_backup_root/$run_id"
  mv "$runtime_mission_dir" "$runtime_failed_backup"
  echo "Preserved incomplete runtime mission: $runtime_failed_backup"
fi

mkdir -p "$task_dir" "$planning_dir"
echo "Safety scope: offline task parsing/planning only; no ROS or flight process is started."

if [[ -n "$task_graph_source" ]]; then
  task_graph_source="$(realpath "$task_graph_source")"
  [[ -s "$task_graph_source" ]] || { echo "Missing task graph: $task_graph_source" >&2; exit 2; }
  cp "$task_graph_source" "$task_graph"
else
  parser_args=(
    "$root/scripts/parse_stage2_instruction.py" "$instruction"
    --scene-graph "$scene_graph" --output "$task_graph" --budget-cny "$budget_cny"
  )
  if [[ "$no_cache" -eq 1 ]]; then parser_args+=(--no-cache); fi
  "$python" "${parser_args[@]}"
fi

if [[ "$static_demo" -eq 1 ]]; then
  "$python" "$root/scripts/make_static_demo_task_graph.py" \
    --input "$task_graph" --output "$task_graph.static"
  mv "$task_graph.static" "$task_graph"
  echo "Static demo mode: conditional branch removed; cup is assumed to be on the refrigerator."
fi

if [[ ${#start_values[@]} -eq 0 ]]; then
  read -r sx sy sz syaw < <(
    "$python" -c 'import json,sys; print(*json.load(open(sys.argv[1]))["start_xyz_yaw"])' "$planning_start"
  )
  start_values=("$sx" "$sy" "$sz" "$syaw")
  start_source="planning_start_preview"
else
  start_source="explicit_approved_pose"
fi

planner_args=(
  "$root/scripts/plan_stage2_mission.py"
  --task-graph "$task_graph" \
  --scene-graph "$scene_graph" \
  --voxel-snapshot "$voxel_snapshot" \
  --planning-config "$planning_config" \
  --motion-cache "$planning_dir/motion_cost_cache.json" \
  --start "${start_values[@]}" \
  --max-candidates "$max_candidates" \
  --output-dir "$planning_dir"
)
if [[ "$visit_all_candidates" -eq 1 ]]; then
  planner_args+=(--visit-all-candidates)
fi
"$python" "${planner_args[@]}"
mv "$planning_dir/mission_plan.json" "$raw_mission"

"$python" "$script_dir/finalize_real_mission.py" \
  --mission "$raw_mission" --task-graph "$task_graph" \
  --voxel-snapshot "$voxel_snapshot" --planning-config "$planning_config" \
  --start-source "$start_source" \
  --output-mission "$mission" --output-audit "$audit"

bundle_eligible="$("$python" -c 'import json,sys; print("true" if json.load(open(sys.argv[1]))["motion_preview_bundle_eligible"] else "false")' "$audit")"
if [[ "$bundle_eligible" == "true" ]]; then
  "$python" "$script_dir/export_full_smooth_route.py" \
    --mission "$mission" \
    --output "$full_smooth_route" \
    --path-source clearance_optimized \
    --speed "$speed_mps"
  "$root/real_fly/stage2_runtime/scripts/prepare_real_execution.sh" \
    "$run_id" "$mission" "$task_id" "$voxel_snapshot/metadata.json"
  runtime_mission_dir="$(dirname "$runtime_bundle")"
  cp -a "$full_smooth_route" "$runtime_mission_dir/full_smooth_route.txt"
  bash "$script_dir/generate_final_minco.sh" "$run_id" "$task_id" \
    "$collision_clearance" clearance_optimized "$speed_mps" "$max_yaw_rate"
else
  echo "Dynamic task preserved for preview, but no execution bundle was generated."
  echo "Reason: conditional/recovery motion requires the online perception/outcome executor."
fi

"$python" - "$task_dir/run_manifest.json" "$run_id" "$task_id" "$start_source" \
  "$task_graph" "$mission" "$audit" "$runtime_bundle" "$bundle_eligible" \
  "$full_smooth_route" "$scene_graph" "$planning_config" "$voxel_snapshot" \
  "$collision_clearance" "$speed_mps" "$max_yaw_rate" <<'PY'
import json
import pathlib
import sys

target = pathlib.Path(sys.argv[1])
bundle = pathlib.Path(sys.argv[8])
full_smooth_route = pathlib.Path(sys.argv[10])
document = {
    "format": "pre_map_vln.real_stage2_task_run.v1",
    "run_id": sys.argv[2],
    "task_id": sys.argv[3],
    "start_source": sys.argv[4],
    "task_graph": str(pathlib.Path(sys.argv[5]).resolve()),
    "mission_plan": str(pathlib.Path(sys.argv[6]).resolve()),
    "mission_audit": str(pathlib.Path(sys.argv[7]).resolve()),
    "execution_bundle": str(bundle.resolve()) if bundle.is_file() else None,
    "motion_preview_bundle_eligible": sys.argv[9] == "true",
    "full_smooth_route": (
        str(full_smooth_route.resolve()) if full_smooth_route.is_file() else None
    ),
    "scene_graph": str(pathlib.Path(sys.argv[11]).resolve()),
    "planning_config": str(pathlib.Path(sys.argv[12]).resolve()),
    "voxel_snapshot": str(pathlib.Path(sys.argv[13]).resolve()),
    "collision_clearance_m": float(sys.argv[14]),
    "speed_mps": float(sys.argv[15]),
    "max_yaw_rate_rad_s": float(sys.argv[16]),
    "autonomous_semantic_execution_eligible": False,
    "safety_scope": "offline_preview_only",
}
target.write_text(json.dumps(document, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
PY

echo "Real Stage2 task prepared: $task_dir"
echo "RViz preview: $script_dir/view_real_scene_rviz.sh $run_id 1 $task_id"
if [[ -s "$runtime_bundle" ]]; then
  echo "Preview-only execution bundle: $runtime_bundle"
fi
if [[ -s "$full_smooth_route" ]]; then
  echo "Full-smooth route: $full_smooth_route"
  echo "NX runtime route: $(dirname "$runtime_bundle")/full_smooth_route.txt"
fi
