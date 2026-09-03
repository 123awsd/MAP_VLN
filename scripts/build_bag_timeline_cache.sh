#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root_dir/scripts/lib/docker.sh"
bag_host="$(realpath "${1:?usage: $0 BAG OUTPUT_NPZ [STEPS]}")"
output_arg="${2:?usage: $0 BAG OUTPUT_NPZ [STEPS]}"
steps="${3:-101}"
if [[ "$output_arg" = /* ]]; then output_host="$output_arg"; else output_host="$root_dir/$output_arg"; fi
case "$bag_host" in
  "$root_dir"/outputs/*) bag_container="/workspace/shared/outputs/${bag_host#"$root_dir"/outputs/}" ;;
  *) echo "bag must be under $root_dir/outputs: $bag_host" >&2; exit 2 ;;
esac
case "$output_host" in
  "$root_dir"/outputs/*) output_container="/workspace/shared/outputs/${output_host#"$root_dir"/outputs/}" ;;
  *) echo "output must be under $root_dir/outputs: $output_host" >&2; exit 2 ;;
esac
mkdir -p "$(dirname "$output_host")"
cd "$root_dir"
pre_map_vln_resolve_docker
exec "${docker_cmd[@]}" compose run --rm falcon python3 \
  /workspace/shared/scripts/extract_bag_exploration_timeline.py \
  "$bag_container" "$output_container" --steps "$steps"
