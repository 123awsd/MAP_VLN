#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
instruction="${1:-先检查卧室床边的灯，然后再观察客厅电视；如果没有发现电视，就去柜子旁检查。门和椅子的检查顺序没有要求。}"
run_name="${2:-hm3d_example}"
run_dir="$root_dir/outputs/stage2/$run_name"
scene_graph="$root_dir/outputs/scene_graph/hm3d_stage1_complete_v3.json"

cd "$root_dir"
mkdir -p "$run_dir"
.envs/habitat/bin/python scripts/fuse_rooms_boxes.py \
  outputs/occusg/hm3d_stage1_complete_v3/regions.json \
  outputs/boxer/hm3d_stage1_complete_v3/boxer_3dbbs_fused.csv \
  "$scene_graph"
./scripts/parse_stage2_instruction.py "$instruction" \
  --scene-graph "$scene_graph" \
  --output "$run_dir/task_graph.json"
./scripts/plan_stage2_mission.py \
  --task-graph "$run_dir/task_graph.json" \
  --scene-graph "$scene_graph" \
  --grid-prefix runtime/occusg/hm3d_stage1_complete_v3_grid \
  --start 0 0 1 0 \
  --spread-target-rooms \
  --output-dir "$run_dir"
.envs/habitat/bin/python scripts/run_stage2_habitat.py \
  --task-graph "$run_dir/task_graph.json" \
  --candidates "$run_dir/candidates.json" \
  --scene-graph "$scene_graph" \
  --grid-prefix runtime/occusg/hm3d_stage1_complete_v3_grid \
  --output-dir "$run_dir/habitat_demo" \
  --save-every 1 \
  --minimum-pixels-by-task '{"observe_living_room_tv":10000}'
docker compose run --rm falcon \
  python3 /workspace/falcon_ws/src/pre_map_bridge/scripts/build_stage2_bag.py \
  --source-bag /workspace/shared/outputs/bags/hm3d_stage1_complete_v3_final.bag \
  --execution "/workspace/shared/outputs/stage2/$run_name/habitat_demo/habitat_execution.json" \
  --task-graph "/workspace/shared/outputs/stage2/$run_name/task_graph.json" \
  --candidates "/workspace/shared/outputs/stage2/$run_name/candidates.json" \
  --scene-graph /workspace/shared/outputs/scene_graph/hm3d_stage1_complete_v3.json \
  --frames-dir "/workspace/shared/outputs/stage2/$run_name/habitat_demo/frames" \
  --output-bag /workspace/shared/outputs/bags/hm3d_stage2_complete.bag \
  --hz 5
./scripts/validate_stage2.py --run-dir "$run_dir" --execution-dir "$run_dir/habitat_demo"
