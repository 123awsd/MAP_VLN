#!/usr/bin/env bash
# Natural-language task -> 3-D mission -> collision-checked preview bundle.
# Offline only: this script never starts ROS, hardware, SUPER, or PX4Ctrl.
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
no_cache=0
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

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
stage2_dir="$(cd -- "$script_dir/.." && pwd)"
root="$(cd -- "$stage2_dir/../.." && pwd)"
python="$root/.envs/habitat/bin/python"
run_data="$stage2_dir/data/$run_id"
scene_graph="$run_data/scene_graph.json"
voxel_snapshot="$run_data/voxel_snapshot"
planning_config="$stage2_dir/config/uav_3d_planning_real.yaml"
planning_start="$run_data/planning_start.json"
map_pcd="$run_data/fastlio_complete/handheld_map_${run_id}_complete.pcd"
task_dir="$run_data/tasks/$task_id"
task_graph="$task_dir/task_graph.json"
planning_dir="$task_dir/planning"
raw_mission="$planning_dir/mission_plan_astar.json"
mission="$planning_dir/mission_plan.json"
audit="$task_dir/mission_audit.json"
runtime_bundle="$root/real_fly/stage2_runtime/missions/$run_id/$task_id/execution_bundle.json"

for path in "$python" "$scene_graph" "$voxel_snapshot/metadata.json" \
  "$voxel_snapshot/voxel_map.npz" "$planning_config" "$planning_start" "$map_pcd"; do
  [[ -s "$path" ]] || { echo "Missing Stage2 input: $path" >&2; exit 2; }
done
[[ ! -e "$task_dir" ]] || { echo "Task output already exists; refusing to overwrite: $task_dir" >&2; exit 2; }
[[ ! -e "$runtime_bundle" ]] || { echo "Runtime bundle already exists; refusing to overwrite: $runtime_bundle" >&2; exit 2; }

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

if [[ ${#start_values[@]} -eq 0 ]]; then
  read -r sx sy sz syaw < <(
    "$python" -c 'import json,sys; print(*json.load(open(sys.argv[1]))["start_xyz_yaw"])' "$planning_start"
  )
  start_values=("$sx" "$sy" "$sz" "$syaw")
  start_source="planning_start_preview"
else
  start_source="explicit_approved_pose"
fi

"$python" "$root/scripts/plan_stage2_mission.py" \
  --task-graph "$task_graph" \
  --scene-graph "$scene_graph" \
  --voxel-snapshot "$voxel_snapshot" \
  --planning-config "$planning_config" \
  --motion-cache "$planning_dir/motion_cost_cache.json" \
  --start "${start_values[@]}" \
  --max-candidates "$max_candidates" \
  --output-dir "$planning_dir"
mv "$planning_dir/mission_plan.json" "$raw_mission"

"$python" "$script_dir/finalize_real_mission.py" \
  --mission "$raw_mission" --task-graph "$task_graph" \
  --voxel-snapshot "$voxel_snapshot" --planning-config "$planning_config" \
  --start-source "$start_source" \
  --output-mission "$mission" --output-audit "$audit"

bundle_eligible="$("$python" -c 'import json,sys; print("true" if json.load(open(sys.argv[1]))["motion_preview_bundle_eligible"] else "false")' "$audit")"
if [[ "$bundle_eligible" == "true" ]]; then
  "$root/real_fly/stage2_runtime/scripts/prepare_real_execution.sh" \
    "$run_id" "$mission" "$task_id"
else
  echo "Dynamic task preserved for preview, but no execution bundle was generated."
  echo "Reason: conditional/recovery motion requires the online perception/outcome executor."
fi

"$python" - "$task_dir/run_manifest.json" "$run_id" "$task_id" "$start_source" \
  "$task_graph" "$mission" "$audit" "$runtime_bundle" "$bundle_eligible" <<'PY'
import json
import pathlib
import sys

target = pathlib.Path(sys.argv[1])
bundle = pathlib.Path(sys.argv[8])
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
