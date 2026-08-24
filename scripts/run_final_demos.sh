#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
scene_graph="$root_dir/outputs/scene_graph/hm3d_stage1_complete_v3.json"
grid_prefix="$root_dir/runtime/occusg/hm3d_stage1_complete_v3_grid"
demo_root="$root_dir/outputs/final_demos"
source_bag="/workspace/shared/outputs/bags/hm3d_stage1_complete_v3_final.bag"

cd "$root_dir"
.envs/habitat/bin/python scripts/prepare_final_demos.py \
  --scene-graph "$scene_graph" --grid-prefix "$grid_prefix" --output-root "$demo_root"

for name in conditional_01_branch conditional_02_skip conditional_03_parallel recovery_01_tv recovery_02_lamp recovery_03_exhausted; do
  run_dir="$demo_root/$name"
  stale="$(.envs/habitat/bin/python -c 'import json,sys; print(json.dumps(json.load(open(sys.argv[1]))["controlled_stale_object_ids"]))' "$run_dir/demo_config.json")"
  found="$(.envs/habitat/bin/python -c 'import json,sys; print(json.dumps(json.load(open(sys.argv[1]))["controlled_found_object_ids"]))' "$run_dir/demo_config.json")"
  recovery_args=()
  candidates_for_bag="/workspace/shared/outputs/final_demos/$name/candidates.json"
  if [[ "$name" == recovery_* ]]; then
    recovery_args=(--semantic-recovery --max-recovery-hypotheses 3 --max-recovery-visits 8 --max-viewpoints-per-location 2 --recovery-budget-cny 20)
    candidates_for_bag="/workspace/shared/outputs/final_demos/$name/habitat_demo/candidates_with_recovery.json"
  fi
  .envs/habitat/bin/python scripts/run_stage2_habitat.py \
    --task-graph "$run_dir/task_graph.json" --candidates "$run_dir/candidates.json" \
    --scene-graph "$scene_graph" --grid-prefix "$grid_prefix" \
    --output-dir "$run_dir/habitat_demo" --verification-mode semantic --save-every 1 \
    --max-viewpoint-attempts 3 --controlled-stale-object-ids "$stale" \
    --controlled-found-object-ids "$found" "${recovery_args[@]}"
  docker compose run --rm falcon \
    python3 /workspace/falcon_ws/src/pre_map_bridge/scripts/build_stage2_bag.py \
    --source-bag "$source_bag" \
    --execution "/workspace/shared/outputs/final_demos/$name/habitat_demo/habitat_execution.json" \
    --task-graph "/workspace/shared/outputs/final_demos/$name/task_graph.json" \
    --candidates "$candidates_for_bag" \
    --scene-graph /workspace/shared/outputs/scene_graph/hm3d_stage1_complete_v3.json \
    --frames-dir "/workspace/shared/outputs/final_demos/$name/habitat_demo/frames" \
    --output-bag "/workspace/shared/outputs/bags/final_demos/$name.bag" --hz 5
done

.envs/habitat/bin/python scripts/validate_final_demos.py
