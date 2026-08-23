#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
bag_name="${1:-hm3d_stage1_scan_complete_final}"
loop="${2:-false}"
rate="${3:-1.0}"
topdown="${4:-true}"
roofless_min_z="${5:-0.50}"
roofless_max_z="${6:-2.25}"
rviz_container="pre-map-vln-rviz-${BASHPID}"

cd "$root_dir"
"$root_dir/scripts/prepare_rviz_xauth.sh"
exec docker compose run --rm --name "$rviz_container" falcon \
  roslaunch pre_map_bridge visualization_replay.launch \
  bag_path:="/workspace/shared/outputs/bags/${bag_name}.bag" \
  loop:="$loop" \
  rate:="$rate" \
  topdown:="$topdown" \
  roofless_min_z:="$roofless_min_z" \
  roofless_max_z:="$roofless_max_z"
