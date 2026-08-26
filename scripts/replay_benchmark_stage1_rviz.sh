#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
benchmark_root="${PRE_MAP_VLN_BENCHMARK_ROOT:-/shared/PRE_MAP_VLN_benchmark_v1}"
scene_id="${1:-Z6MFQCViBuw}"
loop="${2:-false}"
rate="${3:-1.0}"
topdown="${4:-false}"
roofless_floor_clearance="${5:-0.25}"
roofless_ceiling_min_height="${6:-1.65}"
roofless_ceiling_thickness="${7:-0.60}"
paper_view="${8:-false}"
bag="$benchmark_root/bags/${scene_id}_stage1_final.bag"
scene_graph="$benchmark_root/maps/scene_graph/${scene_id}.json"
rviz_container="pre-map-vln-stage1-${scene_id}-${BASHPID}"

if [[ ! "$scene_id" =~ ^[A-Za-z0-9]+$ ]]; then
  echo "invalid scene id: $scene_id" >&2
  exit 2
fi
if [[ ! -f "$bag" ]]; then
  echo "Stage-1 bag not found: $bag" >&2
  exit 3
fi
if [[ ! -f "$scene_graph" ]]; then
  echo "scene graph not found: $scene_graph" >&2
  exit 4
fi

cd "$root_dir"
"$root_dir/scripts/prepare_rviz_xauth.sh"
exec env PRE_MAP_VLN_BENCHMARK_ROOT="$benchmark_root" docker compose run --rm \
  --name "$rviz_container" falcon \
  roslaunch pre_map_bridge visualization_replay.launch \
  bag_path:="/workspace/benchmark/bags/${scene_id}_stage1_final.bag" \
  loop:="$loop" \
  rate:="$rate" \
  topdown:="$topdown" \
  paper_view:="$paper_view" \
  scene_graph_path:="/workspace/benchmark/maps/scene_graph/${scene_id}.json" \
  roofless_floor_clearance:="$roofless_floor_clearance" \
  roofless_ceiling_min_height:="$roofless_ceiling_min_height" \
  roofless_ceiling_thickness:="$roofless_ceiling_thickness"
