#!/usr/bin/env bash
set -euo pipefail

episode_name="${1:-hm3d_panorama}"
decomp_threshold="${2:-1.5}"
grid_path="${3:-/workspace/shared/occusg/${episode_name}_grid.npy}"
metadata_path="${4:-/workspace/shared/occusg/${episode_name}_grid.json}"
output_path="${5:-/workspace/shared/outputs/occusg/$episode_name/regions.json}"
log_path="${6:-/workspace/shared/outputs/occusg/$episode_name/occusg.log}"
exec docker compose run --rm occusg \
  bash /workspace/bridge/run_segmentation.sh "$episode_name" "$decomp_threshold" \
  "$grid_path" "$metadata_path" "$output_path" "$log_path"
