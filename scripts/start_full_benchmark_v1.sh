#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
benchmark_root="${PRE_MAP_VLN_BENCHMARK_ROOT:-/shared/PRE_MAP_VLN_benchmark_v1}"
log_dir="$benchmark_root/logs"
run_dir="$benchmark_root/runs"
pid_file="$run_dir/full_benchmark.pid"
mkdir -p "$log_dir" "$run_dir"

if [[ -f "$pid_file" ]]; then
  old_pid="$(<"$pid_file")"
  if [[ "$old_pid" =~ ^[0-9]+$ ]] && kill -0 "$old_pid" 2>/dev/null; then
    echo "benchmark already running as PID $old_pid"
    exit 0
  fi
fi

timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
log_file="$log_dir/full_benchmark_${timestamp}.log"
cd "$root_dir"
nohup env \
  PRE_MAP_VLN_BENCHMARK_ROOT="$benchmark_root" \
  PRE_MAP_VLN_QWEN_BUDGET_CNY="${PRE_MAP_VLN_QWEN_BUDGET_CNY:-20}" \
  "$root_dir/scripts/run_full_benchmark_v1.sh" \
  >"$log_file" 2>&1 </dev/null &
pid="$!"
printf '%s\n' "$pid" >"$pid_file"
printf '%s\n' "$log_file" >"$run_dir/full_benchmark.latest_log"
echo "started PID $pid; log=$log_file"
