#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
benchmark_root="${PRE_MAP_VLN_BENCHMARK_ROOT:-/shared/PRE_MAP_VLN_hm3d7_v2}"

export PRE_MAP_VLN_BENCHMARK_ROOT="$benchmark_root"
export PRE_MAP_VLN_BENCHMARK_SELECTION="${PRE_MAP_VLN_BENCHMARK_SELECTION:-$root_dir/config/hm3d7_v2.json}"
export PRE_MAP_VLN_BENCHMARK_SURVEY="${PRE_MAP_VLN_BENCHMARK_SURVEY:-$benchmark_root/scenes/hm3d7_survey.json}"

exec "$root_dir/scripts/run_benchmark_tasks_v1.sh"
