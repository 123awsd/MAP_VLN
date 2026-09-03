#!/usr/bin/env bash
set -euo pipefail
root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root_dir/scripts/lib/docker.sh"
cache_arg="${1:-$root_dir/outputs/paper_visualization/00337_L2_dense_depth_pointcloud_cloud.npz}"
if [[ "$cache_arg" = /* ]]; then cache="$(realpath "$cache_arg")"; else cache="$(realpath "$root_dir/$cache_arg")"; fi
case "$cache" in
  "$root_dir"/outputs/*) container_cache="/workspace/shared/outputs/${cache#"$root_dir"/outputs/}" ;;
  *) echo "cloud cache must be under $root_dir/outputs" >&2; exit 2 ;;
esac
cd "$root_dir"
"$root_dir/scripts/prepare_rviz_xauth.sh"
pre_map_vln_resolve_docker
exec "${docker_cmd[@]}" compose run --rm --name "pre-map-vln-depth-cloud-${BASHPID}" falcon \
  roslaunch pre_map_bridge depth_cloud_view.launch cloud_cache:="$container_cache"
