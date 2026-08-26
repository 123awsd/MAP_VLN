#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
benchmark_root="${PRE_MAP_VLN_BENCHMARK_ROOT:-/shared/PRE_MAP_VLN_benchmark_v1}"
scenes=("$@")
if (( ${#scenes[@]} == 0 )); then
  echo "usage: audit_benchmark_stage1_bags.sh SCENE..." >&2
  exit 2
fi

cd "$root_dir"
for scene_id in "${scenes[@]}"; do
  if [[ ! -f "$benchmark_root/bags/${scene_id}_stage1_final.bag" ]]; then
    echo "SKIP audit $scene_id: final bag missing" >&2
    continue
  fi
  PRE_MAP_VLN_BENCHMARK_ROOT="$benchmark_root" docker compose run --rm falcon \
    python3 /workspace/falcon_ws/src/pre_map_bridge/scripts/audit_stage1_bag.py \
    --bag "/workspace/benchmark/bags/${scene_id}_stage1_final.bag" \
    --output "/workspace/benchmark/runs/stage1/${scene_id}/bag_audit.json"
done
