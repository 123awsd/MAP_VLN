#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
episode_name="${1:-hm3d_stage1}"
episode_dir="data/episodes/$episode_name"

cd "$root_dir"
.envs/habitat/bin/python scripts/build_occusg_grid.py \
  "$episode_dir" "runtime/occusg/${episode_name}_grid"
./scripts/run_boxer_habitat.sh "$episode_dir" outputs/boxer
./scripts/run_occusg.sh "$episode_name"
.envs/habitat/bin/python scripts/fuse_rooms_boxes.py \
  "outputs/occusg/$episode_name/regions.json" \
  "outputs/boxer/$episode_name/boxer_3dbbs_fused.csv" \
  "outputs/scene_graph/$episode_name.json"
