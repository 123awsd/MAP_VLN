#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
bag_name="${1:-hm3d_stage2_complete}"
loop="${2:-false}"
rate="${3:-1.0}"
rviz_container="pre-map-vln-stage2-rviz-${BASHPID}"

cd "$root_dir"
"$root_dir/scripts/prepare_rviz_xauth.sh"
exec docker compose run --rm --name "$rviz_container" falcon \
  roslaunch pre_map_bridge stage2_replay.launch \
  bag_path:="/workspace/shared/outputs/bags/${bag_name}.bag" \
  loop:="$loop" \
  rate:="$rate"
