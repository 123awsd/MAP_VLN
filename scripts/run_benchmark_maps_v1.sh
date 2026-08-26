#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
benchmark_root="${PRE_MAP_VLN_BENCHMARK_ROOT:-/shared/PRE_MAP_VLN_benchmark_v1}"
scenes=(
  Z6MFQCViBuw zsNo4HB9uLZ 2azQ1b91cZZ QUCTc6BB5sX EU6Fwq7SyZv
  TbHJrupSAjP X7HyMhZNoso oLBMNvg9in8 x8F5xyUWy9e 8194nk5LbLH
)

mkdir -p "$benchmark_root/runs"
exec 9>"$benchmark_root/runs/maps_phase.lock"
if ! flock -n 9; then
  echo "another map-building process already holds $benchmark_root/runs/maps_phase.lock" >&2
  exit 9
fi

cd "$root_dir"
export PRE_MAP_VLN_BENCHMARK_ROOT="$benchmark_root"
export PRE_MAP_VLN_QWEN_BUDGET_CNY="${PRE_MAP_VLN_QWEN_BUDGET_CNY:-20}"

.envs/habitat/bin/python scripts/check_benchmark_capacity.py \
  --root "$benchmark_root" --minimum-free-gb 60

# The first pass preserves failures and continues, so one difficult scene does
# not prevent the remaining scenes from being mapped. The quality retry stage
# below is the authoritative strict gate.
./scripts/run_benchmark_stage1_batch.sh 600 || true
./scripts/audit_benchmark_stage1_bags.sh "${scenes[@]}"
./scripts/run_stage1_quality_retries.sh "${scenes[@]}"

echo "MAPS_PHASE_PASSED root=$benchmark_root scenes=${#scenes[@]}"
