#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
benchmark_root="${PRE_MAP_VLN_BENCHMARK_ROOT:-/shared/PRE_MAP_VLN_hm3d7_v2}"
selection="${PRE_MAP_VLN_HM3D_SELECTION:-$root_dir/config/hm3d7_v2.json}"
dataset_root="$benchmark_root/scenes/hm3d"
scene_config="$dataset_root/hm3d_annotated_basis.scene_dataset_config.json"
survey="$benchmark_root/scenes/hm3d7_survey.json"
max_duration="${PRE_MAP_VLN_HM3D_MAX_DURATION:-1200}"
split="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["split"])' "$selection")"
mapfile -t scenes < <(python3 -c 'import json,sys; print(*json.load(open(sys.argv[1]))["scenes"],sep="\n")' "$selection")

cd "$root_dir"
export PRE_MAP_VLN_BENCHMARK_ROOT="$benchmark_root"
export PRE_MAP_VLN_QWEN_BUDGET_CNY="${PRE_MAP_VLN_QWEN_BUDGET_CNY:-20}"
mkdir -p "$benchmark_root/logs" "$benchmark_root/runs"

.envs/habitat/bin/python scripts/check_benchmark_capacity.py \
  --root "$benchmark_root" --minimum-free-gb 35
./scripts/download_hm3d7_subset.sh

.envs/habitat/bin/python scripts/survey_benchmark_scenes.py \
  --scene-root "$dataset_root/$split" --scene-config "$scene_config" \
  --scene-ids "${scenes[@]}" --output "$survey" --samples 1500

failures=()
for index in "${!scenes[@]}"; do
  scene_id="${scenes[$index]}"
  scene_path="$(find "$dataset_root/$split/$scene_id" -maxdepth 1 -name '*.basis.glb' -print -quit)"
  seed="$((31 + index * 17))"
  echo "=== HM3D [$((index + 1))/${#scenes[@]}] $scene_id seed=$seed ==="
  if PRE_MAP_VLN_SCENE_PATH="$scene_path" \
     PRE_MAP_VLN_SCENE_CONFIG="$scene_config" \
     PRE_MAP_VLN_SURVEY_FILE="$survey" \
     PRE_MAP_VLN_STAGNATION_WINDOW="${PRE_MAP_VLN_HM3D_STAGNATION_WINDOW:-90}" \
     PRE_MAP_VLN_STAGNATION_RADIUS="${PRE_MAP_VLN_HM3D_STAGNATION_RADIUS:-0.80}" \
     PRE_MAP_VLN_STAGNATION_MIN_ELAPSED="${PRE_MAP_VLN_HM3D_STAGNATION_MIN_ELAPSED:-300}" \
     PRE_MAP_VLN_BENCHMARK_ROOT="$benchmark_root" \
     "$root_dir/scripts/run_benchmark_stage1.sh" "$scene_id" "$max_duration" "$seed" \
     2>&1 | tee "$benchmark_root/logs/stage1_${scene_id}.log"; then
    echo "PASS $scene_id"
  else
    failures+=("$scene_id")
    echo "FAIL $scene_id; evidence preserved and remaining scenes continue" >&2
  fi
done

./scripts/audit_benchmark_stage1_bags.sh "${scenes[@]}"
.envs/habitat/bin/python scripts/evaluate_stage1_quality.py \
  --root "$benchmark_root" --survey "$survey" --scenes "${scenes[@]}" || true
./scripts/export_benchmark_roofless_images.sh "${scenes[@]}"

python3 - "$benchmark_root/runs/hm3d7_maps_report.json" "${failures[@]}" <<'PY'
import json,sys
from pathlib import Path
path=Path(sys.argv[1]); failures=sys.argv[2:]
root=path.parents[1]
statuses=[]
for p in sorted((root/'runs/stage1').glob('*/status.json')):
    statuses.append(json.loads(p.read_text(encoding='utf-8')))
payload={
 'format':'pre_map_vln.hm3d7_maps_run.v1',
 'status':'passed' if not failures and len(statuses)>=7 else 'partial',
 'completed_scene_count':sum(x.get('status')=='passed' for x in statuses),
 'failures':failures,'scenes':statuses,
}
path.write_text(json.dumps(payload,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(payload,ensure_ascii=False))
PY

(( ${#failures[@]} == 0 ))
