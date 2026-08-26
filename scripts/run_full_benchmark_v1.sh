#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
benchmark_root="${PRE_MAP_VLN_BENCHMARK_ROOT:-/shared/PRE_MAP_VLN_benchmark_v1}"
mkdir -p "$benchmark_root/runs"
exec 9>"$benchmark_root/runs/full_benchmark.lock"
if ! flock -n 9; then
  echo "another full benchmark process already holds $benchmark_root/runs/full_benchmark.lock" >&2
  exit 9
fi

cd "$root_dir"
export PRE_MAP_VLN_BENCHMARK_ROOT="$benchmark_root"
export PRE_MAP_VLN_QWEN_BUDGET_CNY="${PRE_MAP_VLN_QWEN_BUDGET_CNY:-20}"
./scripts/run_benchmark_maps_v1.sh
./scripts/run_benchmark_tasks_v1.sh
