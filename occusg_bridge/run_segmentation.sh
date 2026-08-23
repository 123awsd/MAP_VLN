#!/usr/bin/env bash
set -euo pipefail

episode_name="${1:-hm3d_panorama}"
log_path="/workspace/shared/outputs/occusg/$episode_name/occusg.log"
mkdir -p "$(dirname "$log_path")"

ros2 launch incremental_dude_ros2 inc_dude.launch.py \
  use_sim_time:=false decomp_threshold:=1.5 >"$log_path" 2>&1 &
node_pid=$!
trap 'kill "$node_pid" 2>/dev/null || true; wait "$node_pid" 2>/dev/null || true' EXIT

python3 /workspace/bridge/publish_grid_and_capture.py \
  --grid "/workspace/shared/occusg/${episode_name}_grid.npy" \
  --metadata "/workspace/shared/occusg/${episode_name}_grid.json" \
  --output "/workspace/shared/outputs/occusg/$episode_name/regions.json"
