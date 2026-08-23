#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
bag_name="${1:-hm3d_stage1_progressive_final}"
loop="${2:-false}"
rate="${3:-1.0}"

cd "$root_dir"
"$root_dir/scripts/prepare_rviz_xauth.sh"
exec docker compose run --rm --name pre-map-vln-rviz falcon \
  roslaunch pre_map_bridge visualization_replay.launch \
  bag_path:="/workspace/shared/outputs/bags/${bag_name}.bag" \
  loop:="$loop" \
  rate:="$rate"
