#!/usr/bin/env bash
set -euo pipefail

benchmark_root="${PRE_MAP_VLN_BENCHMARK_ROOT:-/shared/PRE_MAP_VLN_hm3d7_v2}"
pid_file="$benchmark_root/runs/hm3d7_tasks.pid"
exit_file="$benchmark_root/runs/hm3d7_tasks.exit_code"
log_file="$benchmark_root/logs/hm3d7_tasks.log"

if [[ -f "$pid_file" ]] && kill -0 "$(<"$pid_file")" 2>/dev/null; then
  echo "state=running pid=$(<"$pid_file")"
elif [[ -f "$exit_file" ]]; then
  code="$(<"$exit_file")"
  if [[ "$code" == "0" ]]; then echo "state=passed"; else echo "state=failed exit_code=$code"; fi
else
  echo "state=not_started"
fi
echo "log=$log_file"
if [[ -f "$benchmark_root/reports/acceptance.json" ]]; then
  python3 - "$benchmark_root/reports/acceptance.json" <<'PY'
import json,sys
data=json.load(open(sys.argv[1]))
print(f"acceptance={data.get('status')} scenes={data.get('scene_count')} episodes={data.get('episode_count')}")
PY
fi
