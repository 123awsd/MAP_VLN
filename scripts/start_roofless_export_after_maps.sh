#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
benchmark_root="${PRE_MAP_VLN_BENCHMARK_ROOT:-/shared/PRE_MAP_VLN_benchmark_v1}"
run_dir="$benchmark_root/runs"
log_dir="$benchmark_root/logs"
pid_file="$run_dir/roofless_export.pid"
exit_file="$run_dir/roofless_export.exit_code"
log_file="$log_dir/roofless_export.log"
mkdir -p "$run_dir" "$log_dir"

if [[ -f "$pid_file" ]]; then
  old_pid="$(<"$pid_file")"
  if [[ "$old_pid" =~ ^[0-9]+$ ]] && kill -0 "$old_pid" 2>/dev/null; then
    echo "roofless export watcher already running as PID $old_pid"
    exit 0
  fi
fi
rm -f "$exit_file"
cd "$root_dir"
nohup env PRE_MAP_VLN_BENCHMARK_ROOT="$benchmark_root" \
  "$root_dir/scripts/export_roofless_after_maps.sh" \
  >"$log_file" 2>&1 </dev/null &
pid=$!
printf '%s\n' "$pid" >"$pid_file"
echo "started roofless export watcher PID $pid; log=$log_file"
