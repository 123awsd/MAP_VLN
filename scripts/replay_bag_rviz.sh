#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
bag_input="${1:-hm3d_stage1_complete_v3_indoor_v1_final}"
loop="${2:-false}"
rate="${3:-1.0}"
topdown="${4:-false}"
roofless_floor_clearance="${5:-0.25}"
roofless_ceiling_min_height="${6:-1.65}"
roofless_ceiling_thickness="${7:-0.60}"
rviz_config="${8:-stage1.rviz}"
delay="${9:-2.0}"
rviz_container="pre-map-vln-rviz-${BASHPID}"

cd "$root_dir"
"$root_dir/scripts/prepare_rviz_xauth.sh"
outputs_root="$root_dir/outputs"
if [[ -f "$bag_input" ]]; then
  bag_host_path="$(realpath "$bag_input")"
  case "$bag_host_path" in
    "$outputs_root"/*)
      bag_path="/workspace/shared/outputs/${bag_host_path#"$outputs_root"/}"
      ;;
    *)
      echo "Bag path must resolve below $outputs_root: $bag_input" >&2
      exit 1
      ;;
  esac
elif [[ -f "$root_dir/outputs/bags/$bag_input/$bag_input.bag" ]]; then
  bag_path="/workspace/shared/outputs/bags/$bag_input/$bag_input.bag"
elif [[ -f "$root_dir/outputs/bags/${bag_input}.bag" ]]; then
  bag_path="/workspace/shared/outputs/bags/${bag_input}.bag"
else
  echo "bag not found by path, grouped name, or legacy name: $bag_input" >&2
  exit 1
fi
exec docker compose run --rm --name "$rviz_container" falcon \
  roslaunch pre_map_bridge visualization_replay.launch \
  bag_path:="$bag_path" \
  loop:="$loop" \
  rate:="$rate" \
  topdown:="$topdown" \
  roofless_floor_clearance:="$roofless_floor_clearance" \
  roofless_ceiling_min_height:="$roofless_ceiling_min_height" \
  roofless_ceiling_thickness:="$roofless_ceiling_thickness" \
  rviz_config:="$rviz_config" \
  delay:="$delay"
