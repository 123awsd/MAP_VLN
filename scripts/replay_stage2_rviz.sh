#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
bag_name="${1:-hm3d_stage2_complete}"
loop="${2:-false}"
rate="${3:-1.0}"
scene_graph_host="${4:-}"
rviz_container="pre-map-vln-stage2-rviz-${BASHPID}"

cd "$root_dir"
"$root_dir/scripts/prepare_rviz_xauth.sh"
scene_graph_arg=""
if [[ -n "$scene_graph_host" ]]; then
  scene_graph_host="$(realpath "$scene_graph_host")"
  case "$scene_graph_host" in
    "$root_dir"/outputs/*) scene_graph_arg="/workspace/shared/outputs/${scene_graph_host#"$root_dir"/outputs/}" ;;
    *) echo "scene graph must be under $root_dir/outputs: $scene_graph_host" >&2; exit 1 ;;
  esac
fi
exec docker compose run --rm --name "$rviz_container" falcon \
  roslaunch pre_map_bridge stage2_replay.launch \
  bag_path:="/workspace/shared/outputs/bags/${bag_name}.bag" \
  scene_graph:="$scene_graph_arg" \
  loop:="$loop" \
  rate:="$rate"
