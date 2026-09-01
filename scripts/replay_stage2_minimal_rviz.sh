#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
bag_name="${1:?usage: $0 <bag-name> <scene-graph>}"
scene_graph_host="${2:?usage: $0 <bag-name> <scene-graph>}"
if [[ $# -ne 2 ]]; then
  echo "usage: $0 <bag-name> <scene-graph>" >&2
  exit 2
fi
container_name="pre-map-vln-stage2-minimal-rviz-${BASHPID}"

if [[ -f "$root_dir/outputs/bags/$bag_name/$bag_name.bag" ]]; then
  bag_path="/workspace/shared/outputs/bags/$bag_name/$bag_name.bag"
elif [[ -f "$root_dir/outputs/bags/${bag_name}.bag" ]]; then
  bag_path="/workspace/shared/outputs/bags/${bag_name}.bag"
else
  echo "bag not found in grouped or legacy layout: $bag_name" >&2
  exit 1
fi

scene_graph_host="$(realpath "$scene_graph_host")"
case "$scene_graph_host" in
  "$root_dir"/outputs/*)
    scene_graph="/workspace/shared/outputs/${scene_graph_host#"$root_dir"/outputs/}"
    ;;
  *)
    echo "scene graph must be below $root_dir/outputs: $scene_graph_host" >&2
    exit 1
    ;;
esac

cd "$root_dir"
"$root_dir/scripts/prepare_rviz_xauth.sh"
exec docker compose run --rm --name "$container_name" falcon \
  roslaunch pre_map_bridge stage2_replay_minimal.launch \
  bag_path:="$bag_path" scene_graph:="$scene_graph"
