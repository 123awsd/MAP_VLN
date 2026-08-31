#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root_dir"

if [[ $# -lt 3 || $# -gt 4 ]]; then
  echo "usage: $0 <stage1-run-dir> <fused-boxes.csv> <prepared-name> [max-floor]" >&2
  exit 2
fi

run_dir="$(realpath -m "$1")"
boxes="$(realpath -m "$2")"
name="$3"
max_floor="${4:-3}"

[[ "$name" =~ ^[A-Za-z0-9_.-]+$ ]] || { echo "invalid prepared name: $name" >&2; exit 2; }
[[ "$max_floor" =~ ^[1-9][0-9]*$ ]] || { echo "max-floor must be positive" >&2; exit 2; }
scene_id="$(basename "$(dirname "$run_dir")")"
[[ "$scene_id" =~ ^[0-9]{5}-[A-Za-z0-9]+$ ]] || {
  echo "cannot infer scene ID from <...>/<scene-id>/<run-name>: $run_dir" >&2; exit 2;
}

required=(
  "$run_dir/generated_3d_config.json"
  "$run_dir/generated_3d_config.yaml"
  "$run_dir/bag_export/map_occupied.pcd"
  "$run_dir/bag_export/map_free.pcd"
  "$run_dir/bag_export/map_unknown.pcd"
  "$run_dir/bag_export/trajectory.csv"
  "$boxes"
  "$root_dir/.secrets/dashscope_api_key"
)
for path in "${required[@]}"; do
  [[ -f "$path" ]] || { echo "missing required input: $path" >&2; exit 2; }
done

floor_dir="$root_dir/outputs/room_pipeline/${name}_floors"
wall_dir="$root_dir/outputs/wall_extraction/$name"
transition_dir="$root_dir/outputs/room_pipeline/${name}_transition"
semantic_dir="$root_dir/outputs/room_pipeline/${name}_semantics"
prepared_dir="$root_dir/outputs/stage2_3d/prepared/$name"
voxel_dir="$prepared_dir/voxel_snapshot"

for target in "$floor_dir" "$wall_dir" "$transition_dir" "$semantic_dir" "$prepared_dir"; do
  [[ ! -e "$target" ]] || { echo "target already exists; refusing to overwrite: $target" >&2; exit 2; }
done

export MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp/pre_map_vln_mpl}"

.envs/habitat/bin/python scripts/run_multifloor_room_pipeline.py \
  "$run_dir" "$boxes" "$floor_dir" \
  --max-floor "$max_floor" --auto-policy \
  --occusg-output-group "$name/floor_preparation"

.envs/habitat/bin/python scripts/extract_multifloor_wall_grid.py \
  "$run_dir/bag_export/map_occupied.pcd" \
  "$run_dir/bag_export/map_free.pcd" \
  "$floor_dir" "$wall_dir" --max-floor "$max_floor"

.envs/habitat/bin/python scripts/run_transition_room_pipeline.py \
  "$run_dir" "$wall_dir" "$transition_dir" \
  --max-floor "$max_floor" --decomp-threshold 1.8 \
  --occusg-output-group "$name/transition_rooms"

.envs/habitat/bin/python scripts/classify_multifloor_rooms_qwen.py \
  "$transition_dir" "$floor_dir" "$wall_dir" "$semantic_dir" \
  --max-floor "$max_floor" --budget-cny "${PRE_MAP_VLN_QWEN_BUDGET_CNY:-20}"

mkdir -p "$prepared_dir"
.envs/habitat/bin/python scripts/prepare_multifloor_stage2_scene.py \
  "$semantic_dir/multifloor_scene_graph_qwen.json" "$wall_dir" \
  "$prepared_dir/scene_graph.json" \
  --transitions "$run_dir/transition_reconstruction/transitions.json"

.envs/habitat/bin/python scripts/build_falcon_voxel_snapshot.py \
  "$run_dir/bag_export" "$run_dir/generated_3d_config.yaml" "$voxel_dir" \
  --planning-config config/uav_3d_planning_habitat.yaml

cp "$run_dir/generated_3d_config.json" "$prepared_dir/generated_stage1_config.json"
.envs/habitat/bin/python - \
  "$prepared_dir/manifest.json" "$name" "$scene_id" "$max_floor" \
  "$run_dir" "$boxes" "$floor_dir" "$wall_dir" "$transition_dir" "$semantic_dir" <<'PY'
import json, pathlib, sys
target = pathlib.Path(sys.argv[1])
document = {
    "format": "pre_map_vln.prepared_multifloor_stage2.v1",
    "name": sys.argv[2],
    "scene_id": sys.argv[3],
    "max_floor": int(sys.argv[4]),
    "source_stage1_run": sys.argv[5],
    "source_fused_boxes": sys.argv[6],
    "scene_graph": "scene_graph.json",
    "voxel_snapshot": "voxel_snapshot",
    "generated_stage1_config": "generated_stage1_config.json",
    "room_outputs": {
        "floors": sys.argv[7], "walls": sys.argv[8],
        "transitions": sys.argv[9], "semantics": sys.argv[10],
    },
}
target.write_text(json.dumps(document, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
PY

echo "Prepared multi-floor Stage2 input: $prepared_dir"
echo "Run a task with: ./scripts/run_prepared_stage2.sh $name <run-name> '<instruction>'"
