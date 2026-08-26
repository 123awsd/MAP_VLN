#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
benchmark_root="${PRE_MAP_VLN_BENCHMARK_ROOT:-/shared/PRE_MAP_VLN_hm3d7_v2}"
run_dir="$benchmark_root/runs"
log_dir="$benchmark_root/logs"
pid_file="$run_dir/hm3d7_maps.pid"
exit_file="$run_dir/hm3d7_maps.exit_code"
log_file="$log_dir/hm3d7_maps.log"

if [[ ! -f "$root_dir/.secrets/hm3d_curl.conf" ]]; then
  echo "First run: .envs/habitat/bin/python scripts/configure_hm3d_credentials.py" >&2
  exit 3
fi
mkdir -p "$run_dir" "$log_dir"
if [[ -f "$pid_file" ]]; then
  old_pid="$(<"$pid_file")"
  if [[ "$old_pid" =~ ^[0-9]+$ ]] && kill -0 "$old_pid" 2>/dev/null; then
    echo "HM3D7 maps already running as PID $old_pid"
    exit 0
  fi
fi
rm -f "$exit_file"
rm -f "$benchmark_root/runs/hm3d7_maps_report.json"
cd "$root_dir"
nohup bash -c '
  set +e
  "$1"
  code=$?
  printf "%s\n" "$code" >"$2"
  exit "$code"
' _ "$root_dir/scripts/run_hm3d7_maps.sh" "$exit_file" \
  >"$log_file" 2>&1 </dev/null &
pid=$!
printf '%s\n' "$pid" >"$pid_file"
echo "started HM3D7 maps PID $pid; log=$log_file"
