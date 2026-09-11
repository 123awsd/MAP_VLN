#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
run_name="${1:-hm3d_vlm}"
cache_mode="${2:-cache}"
run_dir="$root_dir/outputs/stage2/$run_name"
source_run="$root_dir/outputs/stage2/hm3d_example"
extra_args=()
if [[ "$cache_mode" == "no-cache" ]]; then
  extra_args+=(--vlm-no-cache)
fi

cd "$root_dir"
.envs/habitat/bin/python scripts/run_stage2_habitat.py \
  --task-graph "$source_run/task_graph.json" \
  --candidates "$source_run/candidates.json" \
  --scene-graph outputs/scene_graph/hm3d_stage1_complete_v3.json \
  --grid-prefix runtime/occusg/hm3d_stage1_complete_v3_grid \
  --output-dir "$run_dir/habitat_demo" \
  --save-every 1 \
  --verification-mode qwen_vl \
  --planning-strategy rolling_representative \
  --representatives-per-location 1 \
  "${extra_args[@]}"
docker compose run --rm falcon \
  python3 /workspace/falcon_ws/src/pre_map_bridge/scripts/build_stage2_bag.py \
  --source-bag /workspace/shared/outputs/bags/hm3d_stage1_complete_v3_final.bag \
  --execution "/workspace/shared/outputs/stage2/$run_name/habitat_demo/habitat_execution.json" \
  --task-graph /workspace/shared/outputs/stage2/hm3d_example/task_graph.json \
  --candidates /workspace/shared/outputs/stage2/hm3d_example/candidates.json \
  --scene-graph /workspace/shared/outputs/scene_graph/hm3d_stage1_complete_v3.json \
  --frames-dir "/workspace/shared/outputs/stage2/$run_name/habitat_demo/frames" \
  --output-bag /workspace/shared/outputs/bags/hm3d_stage2_vlm_complete.bag \
  --hz 5
python3 scripts/validate_stage2_vlm.py --run-dir "$run_dir"
