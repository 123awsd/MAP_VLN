#!/usr/bin/env bash
set -euo pipefail

benchmark_root="${PRE_MAP_VLN_BENCHMARK_ROOT:-/shared/PRE_MAP_VLN_benchmark_v1}"
phase="${1:-all}"
if [[ "$phase" != "all" && "$phase" != "maps" && "$phase" != "tasks" && "$phase" != "full_benchmark" ]]; then
  echo "usage: benchmark_status.sh [all|maps|tasks|full_benchmark]" >&2
  exit 2
fi

show_phase() {
  local name="$1"
  local pid_file="$benchmark_root/runs/${name}_phase.pid"
  local exit_file="$benchmark_root/runs/${name}_phase.exit_code"
  local log_pointer="$benchmark_root/runs/${name}_phase.latest_log"
  if [[ "$name" == "full_benchmark" ]]; then
    pid_file="$benchmark_root/runs/full_benchmark.pid"
    exit_file="$benchmark_root/runs/full_benchmark.exit_code"
    log_pointer="$benchmark_root/runs/full_benchmark.latest_log"
  fi
  echo "=== phase=$name ==="
  if [[ -f "$pid_file" ]]; then
    local pid
    pid="$(<"$pid_file")"
    if [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null; then
      echo "state=running pid=$pid"
    elif [[ -f "$exit_file" ]]; then
      local code
      code="$(<"$exit_file")"
      if [[ "$code" == "0" ]]; then
        echo "state=passed last_pid=$pid exit_code=0"
      else
        echo "state=failed last_pid=$pid exit_code=$code"
      fi
    else
      echo "state=stopped_without_exit_record last_pid=$pid"
    fi
  else
    echo "state=not_started"
  fi
  if [[ -f "$log_pointer" ]]; then
    local log
    log="$(<"$log_pointer")"
    echo "log=$log"
    [[ -f "$log" ]] && tail -n 30 "$log"
  fi
}

if [[ "$phase" == "all" ]]; then
  show_phase maps
  show_phase tasks
else
  show_phase "$phase"
fi
