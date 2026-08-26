#!/usr/bin/env bash
set -euo pipefail

benchmark_root="${PRE_MAP_VLN_BENCHMARK_ROOT:-/shared/PRE_MAP_VLN_hm3d7_v2}"
pid_file="$benchmark_root/runs/hm3d7_maps.pid"
exit_file="$benchmark_root/runs/hm3d7_maps.exit_code"
log_file="$benchmark_root/logs/hm3d7_maps.log"
if [[ -f "$pid_file" ]]; then
  pid="$(<"$pid_file")"
  if [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null; then
    echo "state=running pid=$pid"
  elif [[ -f "$exit_file" ]]; then
    code="$(<"$exit_file")"
    [[ "$code" == 0 ]] && echo "state=passed exit_code=0" || echo "state=failed exit_code=$code"
  else
    echo "state=stopped_without_exit_record"
  fi
else
  echo "state=not_started"
fi
if [[ -f "$benchmark_root/runs/hm3d7_maps_report.json" ]]; then
  python3 - "$benchmark_root/runs/hm3d7_maps_report.json" <<'PY'
import json,sys
d=json.load(open(sys.argv[1])); print('completed={}/7 failures={}'.format(d['completed_scene_count'],','.join(d['failures']) or 'none'))
PY
fi
[[ -f "$log_file" ]] && { echo "log=$log_file"; tail -n 30 "$log_file"; }
