#!/usr/bin/env bash
set -euo pipefail
root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root_dir/scripts/lib/docker.sh"
bag="/workspace/shared/outputs/bags/trimmed_lz4/hm3d_stage1_3d_00337_online_floor_priority_historyfix_trim420.bag"
output="/workspace/shared/outputs/paper_visualization/00337_online_floor_priority_timeline_101.npz"
mkdir -p "$root_dir/outputs/paper_visualization"
cd "$root_dir"
pre_map_vln_resolve_docker
exec "${docker_cmd[@]}" compose run --rm falcon python3 \
  /workspace/shared/scripts/extract_bag_exploration_timeline.py \
  "$bag" "$output" --steps 101
