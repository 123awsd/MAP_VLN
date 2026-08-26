#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
benchmark_root="${PRE_MAP_VLN_BENCHMARK_ROOT:-/shared/PRE_MAP_VLN_benchmark_v1}"
episode_id="${1:-Z6MFQCViBuw_08}"
loop="${2:-false}"
rate="${3:-1.0}"
rviz_container="pre-map-vln-benchmark-rviz-${BASHPID}"

cd "$root_dir"
"$root_dir/scripts/prepare_rviz_xauth.sh"
exec env PRE_MAP_VLN_BENCHMARK_ROOT="$benchmark_root" docker compose run --rm \
  --name "$rviz_container" falcon \
  roslaunch pre_map_bridge stage2_replay.launch \
  bag_path:="/workspace/benchmark/bags/representative/${episode_id}.bag" \
  loop:="$loop" rate:="$rate"
