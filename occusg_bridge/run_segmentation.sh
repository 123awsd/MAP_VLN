#!/usr/bin/env bash
set -euo pipefail

episode_name="${1:-hm3d_panorama}"
decomp_threshold="${2:-1.5}"
grid_path="${3:-/workspace/shared/occusg/${episode_name}_grid.npy}"
metadata_path="${4:-/workspace/shared/occusg/${episode_name}_grid.json}"
output_path="${5:-/workspace/shared/outputs/occusg/$episode_name/regions.json}"
log_path="${6:-/workspace/shared/outputs/occusg/$episode_name/occusg.log}"
mkdir -p "$(dirname "$log_path")"
mkdir -p "$(dirname "$output_path")"

ros2 launch incremental_dude_ros2 inc_dude.launch.py \
  use_sim_time:=false decomp_threshold:="$decomp_threshold" >"$log_path" 2>&1 &
node_pid=$!
trap 'kill "$node_pid" 2>/dev/null || true; wait "$node_pid" 2>/dev/null || true' EXIT

python3 /workspace/bridge/publish_grid_and_capture.py \
  --grid "$grid_path" \
  --metadata "$metadata_path" \
  --output "$output_path"
