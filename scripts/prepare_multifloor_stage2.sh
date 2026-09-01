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
scene_stage2_dir="$root_dir/outputs/scenes/$scene_id/stage2"
scene_bundle_dir="$scene_stage2_dir/$name"

for target in "$floor_dir" "$wall_dir" "$transition_dir" "$semantic_dir" "$prepared_dir" "$scene_bundle_dir"; do
  [[ ! -e "$target" && ! -L "$target" ]] || {
    echo "target already exists; refusing to overwrite: $target" >&2; exit 2;
  }
done

export MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp/pre_map_vln_mpl}"

.envs/habitat/bin/python scripts/run_multifloor_room_pipeline.py \
  "$run_dir" "$boxes" "$floor_dir" \
  --max-floor "$max_floor" --auto-policy \
  --occusg-output-group "$name/floor_preparation"

actual_floor_count="$(.envs/habitat/bin/python -c \
  'import json,sys; print(len(json.load(open(sys.argv[1]))))' \
  "$floor_dir/floors.json")"
[[ "$actual_floor_count" =~ ^[1-9][0-9]*$ ]] || {
  echo "invalid inferred floor count: $actual_floor_count" >&2; exit 2;
}
echo "Inferred $actual_floor_count floor(s); requested maximum was $max_floor"

.envs/habitat/bin/python scripts/extract_multifloor_wall_grid.py \
  "$run_dir/bag_export/map_occupied.pcd" \
  "$run_dir/bag_export/map_free.pcd" \
  "$floor_dir" "$wall_dir" --max-floor "$actual_floor_count"

.envs/habitat/bin/python scripts/run_transition_room_pipeline.py \
  "$run_dir" "$wall_dir" "$transition_dir" \
  --max-floor "$actual_floor_count" --decomp-threshold 1.8 \
  --occusg-output-group "$name/transition_rooms"

.envs/habitat/bin/python scripts/classify_multifloor_rooms_qwen.py \
  "$transition_dir" "$floor_dir" "$wall_dir" "$semantic_dir" \
  --max-floor "$actual_floor_count" --budget-cny "${PRE_MAP_VLN_QWEN_BUDGET_CNY:-20}"

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
  "$prepared_dir/manifest.json" "$name" "$scene_id" "$actual_floor_count" \
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

# Publish a scene-centric, zero-copy view only after every processing stage
# above has succeeded.  Relative links remain valid when the repository is
# moved as a whole; the temporary directory makes publication atomic.
mkdir -p "$scene_stage2_dir"
bundle_tmp="$scene_stage2_dir/.${name}.tmp.$$"
trap 'rm -rf -- "${bundle_tmp:-}"' EXIT
mkdir "$bundle_tmp"
link_into_bundle() {
  local source="$1" link_name="$2" relative
  relative="$(realpath --relative-to="$bundle_tmp" "$source")"
  ln -s "$relative" "$bundle_tmp/$link_name"
}
link_into_bundle "$run_dir" stage1_run
link_into_bundle "$boxes" fused_boxes.csv
link_into_bundle "$floor_dir" floors
link_into_bundle "$wall_dir" walls
link_into_bundle "$transition_dir" transition_rooms
link_into_bundle "$semantic_dir" semantics
link_into_bundle "$prepared_dir" prepared
.envs/habitat/bin/python - \
  "$bundle_tmp/README.json" "$scene_id" "$name" "$actual_floor_count" <<'PY'
import json, pathlib, sys
path = pathlib.Path(sys.argv[1])
path.write_text(json.dumps({
    "format": "pre_map_vln.scene_stage2_bundle.v1",
    "scene_id": sys.argv[2],
    "name": sys.argv[3],
    "floor_count": int(sys.argv[4]),
    "contents": {
        "stage1_run": "stage1_run",
        "fused_boxes": "fused_boxes.csv",
        "floor_grids": "floors",
        "wall_extraction": "walls",
        "transition_rooms": "transition_rooms",
        "room_semantics": "semantics",
        "prepared_stage2": "prepared",
    },
    "storage": "Relative symlinks to canonical outputs; data is not duplicated.",
}, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
PY
mv "$bundle_tmp" "$scene_bundle_dir"
trap - EXIT

echo "Prepared multi-floor Stage2 input: $prepared_dir"
echo "Scene-centric Stage2 bundle: $scene_bundle_dir"
echo "Run a task with: ./scripts/run_prepared_stage2.sh $name <run-name> '<instruction>'"
