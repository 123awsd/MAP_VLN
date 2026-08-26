#!/usr/bin/env bash
set -euo pipefail

if (( $# != 1 )) || [[ "$1" != "maps" && "$1" != "tasks" ]]; then
  echo "usage: start_benchmark_phase.sh {maps|tasks}" >&2
  exit 2
fi

phase="$1"
root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
benchmark_root="${PRE_MAP_VLN_BENCHMARK_ROOT:-/shared/PRE_MAP_VLN_benchmark_v1}"
log_dir="$benchmark_root/logs"
run_dir="$benchmark_root/runs"
pid_file="$run_dir/${phase}_phase.pid"
exit_file="$run_dir/${phase}_phase.exit_code"
log_pointer="$run_dir/${phase}_phase.latest_log"
mkdir -p "$log_dir" "$run_dir"

if [[ -f "$pid_file" ]]; then
  old_pid="$(<"$pid_file")"
  if [[ "$old_pid" =~ ^[0-9]+$ ]] && kill -0 "$old_pid" 2>/dev/null; then
    echo "$phase phase already running as PID $old_pid"
    exit 0
  fi
fi

timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
log_file="$log_dir/${phase}_phase_${timestamp}.log"
runner="$root_dir/scripts/run_benchmark_${phase}_v1.sh"
rm -f "$exit_file"
cd "$root_dir"
nohup env \
  PRE_MAP_VLN_BENCHMARK_ROOT="$benchmark_root" \
  PRE_MAP_VLN_QWEN_BUDGET_CNY="${PRE_MAP_VLN_QWEN_BUDGET_CNY:-20}" \
  "$root_dir/scripts/run_detached_benchmark_phase.sh" "$runner" "$exit_file" \
  >"$log_file" 2>&1 </dev/null &
pid="$!"
printf '%s\n' "$pid" >"$pid_file"
printf '%s\n' "$log_file" >"$log_pointer"
echo "started phase=$phase pid=$pid log=$log_file"
