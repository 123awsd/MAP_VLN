#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
bag_name="${1:-hm3d_visualization}"
show_rviz="${2:-true}"
bag_path="${3:-/workspace/shared/outputs/bags/${bag_name}.bag}"

cd "$root_dir"
mkdir -p outputs/bags
# A recording must start from a fresh file-bridge session.  Replaying the last
# frame from a previous run before Habitat restarts can make FALCON build a stale
# map and then observe a discontinuous pose jump.
rm -f \
  runtime/bridge/state.json \
  runtime/bridge/depth_u16.raw \
  runtime/bridge/rgb_u8.raw \
  runtime/bridge/semantic_i32.raw \
  runtime/bridge/command.json \
  runtime/bridge/exploration_complete.json \
  runtime/bridge/run_result.json
if [[ "$show_rviz" == "true" ]]; then
  "$root_dir/scripts/prepare_rviz_xauth.sh"
fi
exec docker compose run --rm --name pre-map-vln-falcon-vis falcon \
  roslaunch pre_map_bridge visualization_record.launch \
  bag_path:="$bag_path" \
  rviz:="$show_rviz" \
  topdown:=false
