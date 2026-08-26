#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
benchmark_root="${PRE_MAP_VLN_BENCHMARK_ROOT:-/shared/PRE_MAP_VLN_benchmark_v1}"
selection="${PRE_MAP_VLN_BENCHMARK_SELECTION:-}"
survey="${PRE_MAP_VLN_BENCHMARK_SURVEY:-}"
if [[ -n "$selection" ]]; then
  mapfile -t scenes < <(python3 -c 'import json,sys; print(*json.load(open(sys.argv[1]))["scenes"],sep="\n")' "$selection")
else
  scenes=(
    Z6MFQCViBuw zsNo4HB9uLZ 2azQ1b91cZZ QUCTc6BB5sX EU6Fwq7SyZv
    TbHJrupSAjP X7HyMhZNoso oLBMNvg9in8 x8F5xyUWy9e 8194nk5LbLH
  )
fi
expected_count="$((10 * ${#scenes[@]}))"
representative=("${scenes[0]}_01" "${scenes[0]}_05" "${scenes[0]}_08")

mkdir -p "$benchmark_root/runs"
exec 9>"$benchmark_root/runs/tasks_phase.lock"
if ! flock -n 9; then
  echo "another task-benchmark process already holds $benchmark_root/runs/tasks_phase.lock" >&2
  exit 9
fi

cd "$root_dir"
export PRE_MAP_VLN_BENCHMARK_ROOT="$benchmark_root"
export PRE_MAP_VLN_QWEN_BUDGET_CNY="${PRE_MAP_VLN_QWEN_BUDGET_CNY:-20}"

# Refuse to generate or execute tasks from incomplete/low-quality maps.
quality_args=(--root "$benchmark_root" --scenes "${scenes[@]}")
[[ -n "$survey" ]] && quality_args+=(--survey "$survey")
.envs/habitat/bin/python scripts/evaluate_stage1_quality.py "${quality_args[@]}"

.envs/habitat/bin/python scripts/build_benchmark_v1.py \
  --root "$benchmark_root" --scenes "${scenes[@]}"
.envs/habitat/bin/python scripts/prepare_benchmark_v1.py --root "$benchmark_root"
.envs/habitat/bin/python scripts/check_benchmark_capacity.py \
  --root "$benchmark_root" --minimum-free-gb 20
.envs/habitat/bin/python scripts/run_benchmark_v1.py \
  --root "$benchmark_root" --verification-mode owlv2
.envs/habitat/bin/python scripts/evaluate_benchmark_v1.py \
  --root "$benchmark_root" --require-count "$expected_count"
./scripts/build_benchmark_bags.sh "${representative[@]}"
.envs/habitat/bin/python scripts/evaluate_benchmark_v1.py \
  --root "$benchmark_root" --require-count "$expected_count"
.envs/habitat/bin/python scripts/validate_benchmark_v1.py \
  --root "$benchmark_root" --scene-count "${#scenes[@]}" \
  --episode-count "$expected_count" --representative "${representative[@]}"

echo "TASKS_PHASE_PASSED root=$benchmark_root tasks=$expected_count"
