#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root_dir/scripts/lib/docker.sh"

episode_name="${1:-hm3d_panorama}"
decomp_threshold="${2:-1.5}"
pre_map_vln_resolve_docker
exec "${docker_cmd[@]}" compose run --rm occusg \
  bash /workspace/bridge/run_segmentation.sh "$episode_name" "$decomp_threshold"
