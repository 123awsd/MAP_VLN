#!/usr/bin/env bash
set -uo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
benchmark_root="${PRE_MAP_VLN_BENCHMARK_ROOT:-/shared/PRE_MAP_VLN_benchmark_v1}"
max_duration="${1:-600}"
scenes=(
  Z6MFQCViBuw
  zsNo4HB9uLZ
  2azQ1b91cZZ
  QUCTc6BB5sX
  EU6Fwq7SyZv
  TbHJrupSAjP
  X7HyMhZNoso
  oLBMNvg9in8
  x8F5xyUWy9e
  8194nk5LbLH
)
mkdir -p "$benchmark_root/logs"
failures=()
for index in "${!scenes[@]}"; do
  scene="${scenes[$index]}"
  seed="$((17 + index * 13))"
  echo "=== [$((index + 1))/${#scenes[@]}] $scene seed=$seed ==="
  if PRE_MAP_VLN_BENCHMARK_ROOT="$benchmark_root"       PRE_MAP_VLN_QWEN_BUDGET_CNY="${PRE_MAP_VLN_QWEN_BUDGET_CNY:-20}"       "$root_dir/scripts/run_benchmark_stage1.sh" "$scene" "$max_duration" "$seed"       2>&1 | tee "$benchmark_root/logs/stage1_${scene}.log"; then
    echo "PASS $scene"
  else
    failures+=("$scene")
    echo "FAIL $scene; evidence preserved and batch continues" >&2
  fi
done
python3 - "$benchmark_root/runs/stage1_batch.json" "${failures[@]}" <<'PY'
import json,sys
from pathlib import Path
root=Path(sys.argv[1]).parents[1]
statuses=[]
for path in sorted((root/"runs"/"stage1").glob("*/status.json")):
    statuses.append(json.loads(path.read_text(encoding="utf-8")))
payload={
 "format":"pre_map_vln.benchmark_stage1_batch.v1",
 "status":"passed" if not sys.argv[2:] else "partial",
 "passed":sum(item.get("status")=="passed" for item in statuses),
 "failures":sys.argv[2:],
 "scenes":statuses,
}
Path(sys.argv[1]).write_text(json.dumps(payload,indent=2),encoding="utf-8")
print(json.dumps(payload,ensure_ascii=False))
PY
[[ "${#failures[@]}" -eq 0 ]]
