#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
benchmark_root="${PRE_MAP_VLN_BENCHMARK_ROOT:-/shared/PRE_MAP_VLN_benchmark_v1}"
scenes=("$@")
if (( ${#scenes[@]} == 0 )); then
  echo "usage: run_stage1_quality_retries.sh SCENE..." >&2
  exit 2
fi

cd "$root_dir"
quality_command=(.envs/habitat/bin/python scripts/evaluate_stage1_quality.py \
  --root "$benchmark_root" --scenes "${scenes[@]}")
"${quality_command[@]}" || true

for attempt in 1 2; do
  mapfile -t failed < <(.envs/habitat/bin/python - "$benchmark_root/reports/stage1_quality.json" <<'PY'
import json,sys
for item in json.load(open(sys.argv[1]))["scenes"]:
    if not item["passed"]:
        print(item["scene_id"])
PY
  )
  if (( ${#failed[@]} == 0 )); then
    break
  fi
  echo "Stage-1 quality retry $attempt: ${failed[*]}"
  for scene_id in "${failed[@]}"; do
    scene_index=0
    for index in "${!scenes[@]}"; do
      [[ "${scenes[$index]}" == "$scene_id" ]] && scene_index="$index"
    done
    seed="$((1000 + attempt * 101 + scene_index * 17))"
    if ! PRE_MAP_VLN_STAGNATION_WINDOW=90 \
      PRE_MAP_VLN_STAGNATION_RADIUS=0.40 \
      PRE_MAP_VLN_STAGNATION_MIN_ELAPSED=300 \
      PRE_MAP_VLN_BENCHMARK_ROOT="$benchmark_root" \
        "$root_dir/scripts/run_benchmark_stage1.sh" "$scene_id" 900 "$seed" true; then
      echo "quality retry failed for $scene_id; evidence preserved" >&2
    fi
  done
  "${quality_command[@]}" || true
done

"${quality_command[@]}"
