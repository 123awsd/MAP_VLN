#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
bag_name="${1:-hm3d_stage1_complete_v3_final}"
loop="${2:-false}"
rate="${3:-1.0}"
topdown="${4:-true}"
roofless_floor_clearance="${5:-0.25}"
roofless_ceiling_min_height="${6:-1.65}"
roofless_ceiling_thickness="${7:-0.60}"
rviz_container="pre-map-vln-rviz-${BASHPID}"

cd "$root_dir"
"$root_dir/scripts/prepare_rviz_xauth.sh"
exec docker compose run --rm --name "$rviz_container" falcon \
  roslaunch pre_map_bridge visualization_replay.launch \
  bag_path:="/workspace/shared/outputs/bags/${bag_name}.bag" \
  loop:="$loop" \
  rate:="$rate" \
  topdown:="$topdown" \
  roofless_floor_clearance:="$roofless_floor_clearance" \
  roofless_ceiling_min_height:="$roofless_ceiling_min_height" \
  roofless_ceiling_thickness:="$roofless_ceiling_thickness"
