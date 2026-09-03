#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root_dir/scripts/lib/docker.sh"
default_cache="$root_dir/outputs/paper_visualization/00337_online_floor_priority_timeline_101.npz"
if [[ "${1:-}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  cache_arg="$default_cache"
  initial_percent="$1"
else
  cache_arg="${1:-$default_cache}"
  initial_percent="${2:-35}"
fi
if [[ "$cache_arg" = /* ]]; then cache_host="$cache_arg"; else cache_host="$root_dir/$cache_arg"; fi
cache_host="$(realpath "$cache_host")"
cache_container="/workspace/shared/outputs/paper_visualization/$(basename "$cache_host")"

[[ -f "$cache_host" ]] || {
  echo "timeline cache is missing; run scripts/build_00337_timeline_cache.sh first" >&2
  exit 1
}
cd "$root_dir"
"$root_dir/scripts/prepare_rviz_xauth.sh"
pre_map_vln_resolve_docker
exec "${docker_cmd[@]}" compose run --rm --name "pre-map-vln-timeline-${BASHPID}" falcon \
  roslaunch pre_map_bridge paper_timeline_view.launch \
  timeline_path:="$cache_container" initial_percent:="$initial_percent"
