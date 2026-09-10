#!/usr/bin/env bash
# OccuSG proposal -> manual room editing -> voxel validation -> explicit approval.
# Host-side offline only. This script never starts a flight/control process.
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  review_and_approve_real_scene.sh --run-id ID --min-observations N [options]

Options:
  --decomp-threshold VALUE  OccuSG decomposition threshold (default: 1.8).
  --floor-id N              Semantic floor number (default: 1).
  --floor-z VALUE           Optional floor z used for the 2-D structure projection.
  --room-max-z VALUE        Ignore PCD points above this world z (default: 2.0).
  --wall-min-count N|auto   Minimum points per 5 cm XY cell (default: auto).
  --min-room-area VALUE     Reject room regions smaller than this (default: 5.0 m^2).
  --prepare-only            Generate/reuse the OccuSG proposal, but do not open the editor.
  -h, --help                Show this help.

The Box threshold belongs only to this RUN_ID. The final scene is written only
after validation passes and the terminal receives the exact word APPROVE.
EOF
}

run_id=""
min_observations=""
decomp_threshold="1.8"
floor_id="1"
floor_z=""
room_max_z="2.0"
wall_min_count="auto"
min_room_area="5.0"
prepare_only=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --run-id) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; run_id="$2"; shift 2 ;;
    --min-observations) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; min_observations="$2"; shift 2 ;;
    --decomp-threshold) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; decomp_threshold="$2"; shift 2 ;;
    --floor-id) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; floor_id="$2"; shift 2 ;;
    --floor-z) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; floor_z="$2"; shift 2 ;;
    --room-max-z) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; room_max_z="$2"; shift 2 ;;
    --wall-min-count) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; wall_min_count="$2"; shift 2 ;;
    --min-room-area) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; min_room_area="$2"; shift 2 ;;
    --prepare-only) prepare_only=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ "$run_id" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] || { echo "Invalid --run-id: $run_id" >&2; exit 2; }
[[ "$min_observations" =~ ^[1-9][0-9]*$ ]] || { echo "--min-observations must be a positive integer" >&2; exit 2; }
[[ "$decomp_threshold" =~ ^[0-9]+([.][0-9]+)?$ ]] || { echo "Invalid --decomp-threshold" >&2; exit 2; }
[[ "$floor_id" =~ ^[1-9][0-9]*$ ]] || { echo "Invalid --floor-id" >&2; exit 2; }
if [[ -n "$floor_z" ]]; then
  [[ "$floor_z" =~ ^-?[0-9]+([.][0-9]+)?$ ]] || { echo "Invalid --floor-z" >&2; exit 2; }
fi
[[ "$room_max_z" =~ ^-?[0-9]+([.][0-9]+)?$ ]] || { echo "Invalid --room-max-z" >&2; exit 2; }
[[ "$wall_min_count" == "auto" || "$wall_min_count" =~ ^[1-9][0-9]*$ ]] || { echo "Invalid --wall-min-count" >&2; exit 2; }
[[ "$min_room_area" =~ ^[0-9]+([.][0-9]+)?$ ]] || { echo "Invalid --min-room-area" >&2; exit 2; }

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
stage2_dir="$(cd -- "$script_dir/.." && pwd)"
root="$(cd -- "$stage2_dir/../.." && pwd)"
python="$root/.envs/habitat/bin/python"
run_data="$stage2_dir/data/$run_id"
review_dir="$run_data/room_review_pcd"
map_pcd="$run_data/fastlio_complete/handheld_map_${run_id}_complete.pcd"
trajectory="$run_data/fastlio_complete/trajectory.csv"
clusters="$run_data/localization/target_points_clustered.json"
base_scene="$run_data/scene_graph.json"
voxel_snapshot="$run_data/voxel_snapshot"
planning_start="$run_data/planning_start.json"
final_scene="$run_data/approved_scene_graph.json"
draft="$review_dir/rooms_draft.json"
candidate="$review_dir/approved_scene_graph.candidate.json"
report="$review_dir/validation_report.json"
proposal_manifest="$review_dir/proposal_manifest.json"

for path in "$python" "$map_pcd" "$trajectory" "$clusters" "$base_scene" \
  "$voxel_snapshot/metadata.json" "$voxel_snapshot/voxel_map.npz" "$planning_start"; do
  [[ -s "$path" ]] || { echo "Missing room-review input: $path" >&2; exit 2; }
done
[[ ! -e "$final_scene" ]] || { echo "Approved scene already exists; refusing to overwrite: $final_scene" >&2; exit 2; }

mkdir -p "$review_dir"
threshold_tag="${decomp_threshold//./p}"
grid_key="real_${run_id}_pcd_rooms_d${threshold_tag}"
grid_prefix="$root/runtime/occusg/${grid_key}_grid"
occusg_rel="real_rooms_pcd/$run_id/proposal_d${threshold_tag}"
regions_source="$root/outputs/occusg/$occusg_rel/regions.json"
grid="$review_dir/structure_grid.npy"
grid_metadata="$review_dir/structure_grid.json"
regions="$review_dir/regions_raw.json"

if [[ -s "$proposal_manifest" ]]; then
  "$python" - "$proposal_manifest" "$run_id" "$decomp_threshold" "$floor_id" "$floor_z" "$room_max_z" "$wall_min_count" <<'PY'
import json, sys
document = json.load(open(sys.argv[1], encoding="utf-8"))
expected = {"run_id": sys.argv[2], "decomp_threshold": float(sys.argv[3]),
            "floor_id": int(sys.argv[4]), "floor_z_m": None if not sys.argv[5] else float(sys.argv[5]),
            "room_max_z_m": float(sys.argv[6]), "wall_min_count": sys.argv[7]}
for key, value in expected.items():
    if document.get(key) != value:
        raise SystemExit(f"Existing proposal uses {key}={document.get(key)!r}, requested {value!r}; preserve it or use a new RUN_ID")
PY
  for path in "$grid" "$grid_metadata" "$regions"; do
    [[ -s "$path" ]] || { echo "Incomplete room proposal: $path" >&2; exit 2; }
  done
  current_pcd_sha256="$(sha256sum "$map_pcd" | awk '{print $1}')"
  recorded_pcd_sha256="$("$python" -c 'import json,sys; print(json.load(open(sys.argv[1]))["source_pcd_sha256"])' "$grid_metadata")"
  [[ "$current_pcd_sha256" == "$recorded_pcd_sha256" ]] || {
    echo "The final PCD changed after this room proposal was generated; refusing stale reuse." >&2
    exit 2
  }
  recorded_algorithm="$("$python" -c 'import json,sys; print(json.load(open(sys.argv[1])).get("wall_extraction", {}).get("algorithm", ""))' "$grid_metadata")"
  [[ "$recorded_algorithm" == "bounded_xy_density_boundary_components_v4" ]] || {
    echo "The room-grid algorithm changed; refusing stale proposal reuse." >&2
    exit 2
  }
  echo "Reusing per-run room proposal: $review_dir"
else
  for path in "$grid" "$grid_metadata" "$regions"; do
    [[ ! -e "$path" ]] || { echo "Found proposal artifact without manifest; refusing to overwrite: $path" >&2; exit 2; }
  done
  echo "Safety scope: host-side offline PCD room review only; source PCD and voxel collision data remain read-only."
  grid_args=(
    "$script_dir/build_pcd_room_grid.py" "$map_pcd" "$trajectory" "$grid_prefix"
    --resolution 0.05 --room-max-z "$room_max_z" --wall-min-count "$wall_min_count"
  )
  if [[ -n "$floor_z" ]]; then grid_args+=(--floor-z "$floor_z"); fi
  "$python" "${grid_args[@]}"
  "$root/scripts/run_occusg.sh" "$grid_key" "$decomp_threshold" "$occusg_rel"
  for path in "$grid_prefix.npy" "$grid_prefix.json" "$regions_source"; do
    [[ -s "$path" ]] || { echo "Room proposal generation failed: $path" >&2; exit 2; }
  done
  cp "$grid_prefix.npy" "$grid"
  cp "$grid_prefix.json" "$grid_metadata"
  cp "$grid_prefix.png" "$review_dir/structure_grid.png"
  cp "$regions_source" "$regions"
  "$python" - "$proposal_manifest" "$run_id" "$decomp_threshold" "$floor_id" "$floor_z" \
    "$min_observations" "$map_pcd" "$regions_source" "$room_max_z" "$wall_min_count" <<'PY'
import json, pathlib, sys
target = pathlib.Path(sys.argv[1])
document = {
    "format": "pre_map_vln.real_room_proposal.v1", "run_id": sys.argv[2],
    "decomp_threshold": float(sys.argv[3]), "floor_id": int(sys.argv[4]),
    "floor_z_m": None if not sys.argv[5] else float(sys.argv[5]),
    "initial_min_object_observations": int(sys.argv[6]),
    "source_pcd": str(pathlib.Path(sys.argv[7]).resolve()),
    "source_regions": str(pathlib.Path(sys.argv[8]).resolve()),
    "room_max_z_m": float(sys.argv[9]), "wall_min_count": sys.argv[10],
    "uses_rgbd": False, "uses_qwen": False, "voxel_collision_map_modified": False,
}
temporary = target.with_suffix(target.suffix + ".tmp")
temporary.write_text(json.dumps(document, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
temporary.replace(target)
PY
fi

echo "Room proposal ready: $regions"
if [[ "$prepare_only" -eq 1 ]]; then
  echo "Preparation only; no editor or approval was started."
  exit 0
fi
[[ -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ]] || {
  echo "No graphical display detected. Run this command in a desktop terminal for the room editor." >&2
  exit 2
}

"$python" "$script_dir/edit_real_rooms.py" \
  --run-id "$run_id" --grid "$grid" --metadata "$grid_metadata" --regions "$regions" \
  --clusters "$clusters" --voxel-metadata "$voxel_snapshot/metadata.json" \
  --planning-start "$planning_start" \
  --draft "$draft" --preview "$review_dir/rooms_draft.png" \
  --min-observations "$min_observations" --min-room-area "$min_room_area" --floor-id "$floor_id"

set +e
"$python" "$script_dir/compile_approved_rooms.py" \
  --run-id "$run_id" --rooms "$draft" --scene-graph "$base_scene" --clusters "$clusters" \
  --voxel-snapshot "$voxel_snapshot" --planning-start "$planning_start" \
  --min-observations "$min_observations" --min-room-area "$min_room_area" \
  --candidate "$candidate" --report "$report" \
  --preview "$review_dir/validation_preview.png"
validation_status=$?
set -e
if [[ "$validation_status" -ne 0 ]]; then
  echo "Room validation failed. Edit and rerun the same command; draft was preserved: $draft" >&2
  echo "Validation report: $report" >&2
  exit "$validation_status"
fi

echo "Validation passed. Inspect: $review_dir/validation_preview.png"
echo "Type APPROVE to publish this RUN_ID's approved room scene; anything else keeps only the draft."
read -r approval
if [[ "$approval" != "APPROVE" ]]; then
  echo "Not approved. Draft and validation artifacts were preserved; no final scene was created."
  exit 0
fi
[[ ! -e "$final_scene" ]] || { echo "Approved scene appeared during review; refusing to overwrite: $final_scene" >&2; exit 2; }

cp "$candidate" "$final_scene.tmp"
mv "$final_scene.tmp" "$final_scene"
cp "$draft" "$review_dir/rooms_approved.json"
"$python" - "$review_dir/semantic_approval.json" "$run_id" "$min_observations" "$final_scene" "$report" <<'PY'
import datetime, json, pathlib, sys
target = pathlib.Path(sys.argv[1])
document = {
    "format": "pre_map_vln.real_room_approval.v1", "run_id": sys.argv[2],
    "approved_at_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
    "approval_token": "APPROVE", "min_object_observations": int(sys.argv[3]),
    "approved_scene_graph": str(pathlib.Path(sys.argv[4]).resolve()),
    "validation_report": str(pathlib.Path(sys.argv[5]).resolve()),
    "uses_qwen_for_room_semantics": False, "authorizes_flight": False,
}
target.write_text(json.dumps(document, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
PY
echo "Approved per-run scene graph: $final_scene"
echo "This approval validates offline room semantics only; it does not authorize arming or flight."
