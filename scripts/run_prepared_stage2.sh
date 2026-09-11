#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root_dir"

if [[ $# -lt 3 || $# -gt 4 ]]; then
  echo "usage: $0 <prepared-name> <run-name> <instruction|task-graph.json> [verification-mode]" >&2
  exit 2
fi

prepared_name="$1"
run_name="$2"
task_source="$3"
verification_mode="${4:-owlv2}"
prepared_dir="$root_dir/outputs/stage2_3d/prepared/$prepared_name"
manifest="$prepared_dir/manifest.json"
run_dir="$root_dir/outputs/stage2_3d/tasks/$run_name"

[[ "$run_name" =~ ^[A-Za-z0-9_.-]+$ ]] || { echo "invalid run name: $run_name" >&2; exit 2; }
case "$verification_mode" in semantic|owlv2|qwen_vl|hybrid|owlv2_qwen_fallback|controlled) ;; *)
  echo "unsupported verification mode: $verification_mode" >&2; exit 2;;
esac
for path in "$manifest" "$prepared_dir/scene_graph.json" \
  "$prepared_dir/voxel_snapshot/metadata.json" "$prepared_dir/voxel_snapshot/voxel_map.npz" \
  "$prepared_dir/generated_stage1_config.json"; do
  [[ -e "$path" ]] || { echo "missing prepared input: $path" >&2; exit 2; }
done
[[ ! -e "$run_dir" ]] || { echo "target already exists; refusing to overwrite: $run_dir" >&2; exit 2; }

scene_id="$(.envs/habitat/bin/python - "$manifest" <<'PY'
import json, pathlib, sys
value = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8")).get("scene_id", "")
if not isinstance(value, str) or not value:
    raise SystemExit("prepared manifest has no scene_id")
print(value)
PY
)"
scene_token="${scene_id#*-}"
scene_root="${PRE_MAP_VLN_HM3D_TRAIN_ROOT:-/shared/PRE_MAP_VLN_hm3d7_v2/scenes/hm3d/train}"
scene_config="${PRE_MAP_VLN_HM3D_SCENE_CONFIG:-/shared/PRE_MAP_VLN_hm3d7_v2/scenes/hm3d/hm3d_annotated_basis.scene_dataset_config.json}"
scene="$scene_root/$scene_id/$scene_token.basis.glb"
for path in "$scene" "$scene_config"; do
  [[ -f "$path" ]] || { echo "missing HM3D input: $path" >&2; exit 2; }
done

mkdir -p "$run_dir"
relocated_config="$run_dir/generated_stage1_config.json"
.envs/habitat/bin/python - "$prepared_dir/generated_stage1_config.json" \
  "$relocated_config" "$scene" "$scene_config" <<'PY'
import json, pathlib, sys
source, target, scene, scene_config = map(pathlib.Path, sys.argv[1:])
document = json.loads(source.read_text(encoding="utf-8"))
document["scene"] = str(scene.resolve())
document["scene_config"] = str(scene_config.resolve())
target.write_text(json.dumps(document, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
PY

task_graph="$run_dir/task_graph.json"
if [[ -f "$task_source" ]]; then
  cp "$task_source" "$task_graph"
else
  [[ -f "$root_dir/.secrets/dashscope_api_key" ]] || {
    echo "natural-language parsing needs .secrets/dashscope_api_key; run scripts/configure_secrets.py --qwen-stdin" >&2
    exit 2
  }
  .envs/habitat/bin/python scripts/parse_stage2_instruction.py "$task_source" \
    --scene-graph "$prepared_dir/scene_graph.json" --output "$task_graph" \
    --budget-cny "${PRE_MAP_VLN_QWEN_BUDGET_CNY:-20}"
fi

.envs/habitat/bin/python -u scripts/run_stage2_habitat.py \
  --task-graph "$task_graph" \
  --scene-graph "$prepared_dir/scene_graph.json" \
  --voxel-snapshot "$prepared_dir/voxel_snapshot" \
  --generated-stage1-config "$relocated_config" \
  --planning-config config/uav_3d_planning_habitat.yaml \
  --output-dir "$run_dir/execution" \
  --verification-mode "$verification_mode" \
  --max-candidates 3 --planning-horizon-tasks 2 \
  --planning-strategy rolling_representative --representatives-per-location 1 \
  --owlv2-every 10 --save-every 5

.envs/habitat/bin/python - "$run_dir/run_manifest.json" \
  "$prepared_name" "$scene_id" "$verification_mode" <<'PY'
import json, pathlib, sys
target = pathlib.Path(sys.argv[1])
document = {
    "format": "pre_map_vln.stage2_quickstart_run.v1",
    "prepared_name": sys.argv[2],
    "scene_id": sys.argv[3],
    "verification_mode": sys.argv[4],
    "task_graph": "task_graph.json",
    "execution": "execution/habitat_execution.json",
}
target.write_text(json.dumps(document, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
PY

echo "Stage2 result: $run_dir"
