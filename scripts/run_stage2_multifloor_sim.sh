#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ $# -lt 5 || $# -gt 6 ]]; then
  echo "usage: $0 <task-graph.json> <scene-graph.json> <voxel-snapshot-dir> <generated-stage1-config.json> <run-name> [save-every]" >&2
  exit 2
fi

task_graph="$1"
scene_graph="$2"
voxel_snapshot="$3"
generated_config="$4"
run_name="$5"
save_every="${6:-5}"
run_dir="$root_dir/outputs/stage2_3d/runs/$run_name"

cd "$root_dir"
for path in "$task_graph" "$scene_graph" "$voxel_snapshot/metadata.json" \
  "$voxel_snapshot/voxel_map.npz" "$generated_config"; do
  if [[ ! -e "$path" ]]; then
    echo "missing input: $path" >&2
    exit 2
  fi
done
if [[ -e "$run_dir" ]]; then
  echo "target already exists; refusing to overwrite: $run_dir" >&2
  exit 2
fi

mkdir -p "$run_dir"
.envs/habitat/bin/python scripts/plan_stage2_mission.py \
  --task-graph "$task_graph" \
  --scene-graph "$scene_graph" \
  --voxel-snapshot "$voxel_snapshot" \
  --planning-config config/uav_3d_planning_habitat.yaml \
  --motion-cache "$run_dir/motion_cost_cache.json" \
  --start 0 0 1 0 \
  --spread-target-rooms \
  --output-dir "$run_dir"

.envs/habitat/bin/python scripts/run_stage2_multifloor_habitat.py \
  "$run_dir/mission_plan.json" \
  "$voxel_snapshot" \
  "$generated_config" \
  "$run_dir/habitat" \
  --planning-config config/uav_3d_planning_habitat.yaml \
  --step-m 0.12 \
  --save-every "$save_every"

echo "complete=$run_dir/habitat/habitat_execution.json"
