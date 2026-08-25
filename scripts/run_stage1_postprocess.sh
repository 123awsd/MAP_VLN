#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
episode_name="${1:-hm3d_stage1}"
episode_dir="data/episodes/$episode_name"
semantic_labels="data/scene_datasets/hm3d/example/00861-GLAQ4DNUx5U/GLAQ4DNUx5U.semantic.txt"

cd "$root_dir"
./scripts/run_boxer_habitat.sh "$episode_dir" outputs/boxer
PYTHONPATH="$root_dir/third_party/boxer" .envs/boxer/bin/python \
  third_party/boxer/utils/fuse_3d_boxes.py \
  --input "outputs/boxer/$episode_name/boxer_3dbbs.csv" \
  --output "outputs/boxer/$episode_name/boxer_3dbbs_visual.csv" \
  --iou 0.2 --min_detections 2 --conf_threshold 0.3
.envs/habitat/bin/python scripts/build_occusg_grid.py \
  "$episode_dir" "runtime/occusg/${episode_name}_grid" \
  --semantic-labels "$semantic_labels"
./scripts/run_occusg.sh "$episode_name"
.envs/habitat/bin/python scripts/fuse_rooms_boxes.py \
  "outputs/occusg/$episode_name/regions.json" \
  "outputs/boxer/$episode_name/boxer_3dbbs_fused.csv" \
  "outputs/scene_graph/$episode_name.json" \
  --grid-meta "runtime/occusg/${episode_name}_grid.json"
