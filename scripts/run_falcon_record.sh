#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
bag_name="${1:-hm3d_visualization}"
show_rviz="${2:-true}"

cd "$root_dir"
mkdir -p outputs/bags
if [[ "$show_rviz" == "true" ]]; then
  "$root_dir/scripts/prepare_rviz_xauth.sh"
fi
exec docker compose run --rm --name pre-map-vln-falcon-vis falcon \
  roslaunch pre_map_bridge visualization_record.launch \
  bag_path:="/workspace/shared/outputs/bags/${bag_name}.bag" \
  rviz:="$show_rviz"
