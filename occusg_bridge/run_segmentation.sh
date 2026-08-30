#!/usr/bin/env bash
set -euo pipefail

episode_name="${1:-hm3d_panorama}"
decomp_threshold="${2:-1.5}"
output_rel="${3:-$episode_name}"
case "$output_rel" in
  /*|*..*) echo "invalid OccuSG output path: $output_rel" >&2; exit 2 ;;
esac
output_dir="/workspace/shared/outputs/occusg/$output_rel"
log_path="$output_dir/occusg.log"
mkdir -p "$(dirname "$log_path")"

ros2 launch incremental_dude_ros2 inc_dude.launch.py \
  use_sim_time:=false decomp_threshold:="$decomp_threshold" >"$log_path" 2>&1 &
node_pid=$!
trap 'kill "$node_pid" 2>/dev/null || true; wait "$node_pid" 2>/dev/null || true' EXIT

python3 /workspace/bridge/publish_grid_and_capture.py \
  --grid "/workspace/shared/occusg/${episode_name}_grid.npy" \
  --metadata "/workspace/shared/occusg/${episode_name}_grid.json" \
  --output "$output_dir/regions.json"
