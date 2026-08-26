#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
scene_id="${1:?usage: run_benchmark_stage1.sh SCENE_ID [MAX_DURATION] [SEED]}"
max_duration="${2:-600}"
seed="${3:-17}"
force="${4:-false}"
benchmark_root="${PRE_MAP_VLN_BENCHMARK_ROOT:-/shared/PRE_MAP_VLN_benchmark_v1}"
scene_path="${PRE_MAP_VLN_SCENE_PATH:-$root_dir/data/scene_datasets/mp3d/$scene_id/$scene_id.glb}"
scene_config="${PRE_MAP_VLN_SCENE_CONFIG:-}"
episode_dir="$benchmark_root/episodes/$scene_id"
box_root="$benchmark_root/maps/boxer"
box_dir="$box_root/$scene_id"
stage1_dir="$benchmark_root/maps/stage1/$scene_id"
grid_dir="$benchmark_root/maps/grids"
occusg_dir="$benchmark_root/maps/occusg/$scene_id"
scene_graph_dir="$benchmark_root/maps/scene_graph"
ground_truth_dir="$benchmark_root/scenes/ground_truth"
semantic_dir="$benchmark_root/scenes/semantic_labels"
run_dir="$benchmark_root/runs/stage1/$scene_id"
raw_bag="$benchmark_root/bags/${scene_id}_stage1_raw.bag"
final_bag="$benchmark_root/bags/${scene_id}_stage1_final.bag"
result_file="$run_dir/exploration_result.json"
status_file="$run_dir/status.json"
survey_file="${PRE_MAP_VLN_SURVEY_FILE:-$benchmark_root/scenes/mp3d_survey.json}"

if [[ ! -f "$scene_path" ]]; then
  echo "scene missing: $scene_path" >&2
  exit 2
fi
if [[ "$force" != "true" && -f "$status_file" ]] && python3 -c 'import json,sys; raise SystemExit(0 if json.load(open(sys.argv[1])).get("status")=="passed" else 1)' "$status_file"; then
  echo "$scene_id already passed"
  exit 0
fi
partial_targets=()
for target in \
  "$episode_dir" "$raw_bag" "$final_bag" "$box_dir" "$stage1_dir" "$occusg_dir" \
  "$grid_dir/${scene_id}_navigation.npy" "$grid_dir/${scene_id}_navigation.json" \
  "$grid_dir/${scene_id}_navigation.png" "$grid_dir/${scene_id}_structure.npy" \
  "$grid_dir/${scene_id}_structure.json" "$grid_dir/${scene_id}_structure.png" \
  "$scene_graph_dir/$scene_id.json" "$ground_truth_dir/$scene_id.json" \
  "$semantic_dir/$scene_id.csv"; do
  [[ -e "$target" ]] && partial_targets+=("$target")
done
if (( ${#partial_targets[@]} > 0 )); then
  recovery_id="$(date -u +%Y%m%dT%H%M%SZ)"
  recovery_dir="$benchmark_root/recovery_snapshots/$scene_id/$recovery_id"
  mkdir -p "$recovery_dir"
  printf '%s\n' "Interrupted scene artifacts are archived before a clean resumable rerun." >"$recovery_dir/README.txt"
  for target in "${partial_targets[@]}"; do
    relative="${target#"$benchmark_root"/}"
    archived="$recovery_dir/$relative"
    mkdir -p "$(dirname "$archived")"
    mv "$target" "$archived"
  done
  if [[ -d "$run_dir" ]]; then
    mkdir -p "$recovery_dir/runs/stage1"
    mv "$run_dir" "$recovery_dir/runs/stage1/$scene_id"
  fi
  echo "archived interrupted artifacts at $recovery_dir"
fi

mkdir -p "$episode_dir" "$stage1_dir" "$grid_dir" "$occusg_dir" "$scene_graph_dir"   "$ground_truth_dir" "$semantic_dir" "$run_dir" "$benchmark_root/bags"
cd "$root_dir"
export PRE_MAP_VLN_BENCHMARK_ROOT="$benchmark_root"
floor_height="$(python3 - "$survey_file" "$scene_id" <<'PY'
import json,sys
data=json.load(open(sys.argv[1]))
scene=next(item for item in data["scenes"] if item["scene_id"] == sys.argv[2])
print(scene["floor_clusters"][0]["height_m"])
PY
)"

recorder_pid=""
cleanup() {
  if [[ -n "$recorder_pid" ]] && kill -0 "$recorder_pid" 2>/dev/null; then
    docker kill --signal=SIGINT pre-map-vln-falcon-vis >/dev/null 2>&1 || true
    wait "$recorder_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

./scripts/run_falcon_record.sh "${scene_id}_stage1_raw" false   "/workspace/benchmark/bags/${scene_id}_stage1_raw.bag" >"$run_dir/recorder.log" 2>&1 &
recorder_pid=$!
sleep 3
habitat_scene_args=(--scene "$scene_path" --scene-id "$scene_id")
if [[ -n "$scene_config" ]]; then
  habitat_scene_args+=(--scene-config "$scene_config")
else
  habitat_scene_args+=(--no-scene-config)
fi
.envs/habitat/bin/python scripts/run_habitat_falcon.py   "${habitat_scene_args[@]}" --seed "$seed"   --floor-height "$floor_height" --floor-tolerance 0.40   --duration "$max_duration" --hz 10 --follow-falcon   --idle-scan-rate 25 --idle-scan-after 0.3   --stagnation-window "${PRE_MAP_VLN_STAGNATION_WINDOW:-60}"   --stagnation-radius "${PRE_MAP_VLN_STAGNATION_RADIUS:-0.75}"   --stagnation-min-elapsed "${PRE_MAP_VLN_STAGNATION_MIN_ELAPSED:-180}"   --record-dir "$episode_dir" --record-every 5   --completion-file "$root_dir/runtime/bridge/exploration_complete.json"   --finish-hold 3.0 --result-file "$result_file" | tee "$run_dir/habitat.log"
cleanup
recorder_pid=""
trap - EXIT INT TERM

termination="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["termination"])' "$result_file")"
if [[ "$termination" != "complete" ]]; then
  python3 - "$status_file" "$scene_id" "$termination" <<'PY'
import json,sys
from pathlib import Path
Path(sys.argv[1]).write_text(json.dumps({
    "format":"pre_map_vln.benchmark_stage1_status.v1",
    "scene_id":sys.argv[2],"status":"exploration_incomplete","termination":sys.argv[3],
},indent=2),encoding="utf-8")
PY
  echo "$scene_id exploration did not reach FINISH; raw evidence preserved" >&2
  exit 4
fi

ground_truth_args=(--scene "$scene_path")
[[ -n "$scene_config" ]] && ground_truth_args+=(--scene-config "$scene_config")
.envs/habitat/bin/python scripts/export_mp3d_ground_truth.py   "${ground_truth_args[@]}" --episode-manifest "$episode_dir/manifest.json"   --semantic-labels "$semantic_dir/$scene_id.csv"   --ground-truth "$ground_truth_dir/$scene_id.json"

./scripts/run_boxer_habitat.sh "$episode_dir" "$box_root"   config/stage1_indoor_full_v1.txt full_v1 | tee "$run_dir/boxer.log"

PYTHONPATH="$root_dir/third_party/boxer" .envs/boxer/bin/python   scripts/build_progressive_boxes.py   "$box_dir/full_v1_3dbbs.csv" "$box_dir/full_v1_3dbbs_progressive.csv"   --iou 0.2 --min-detections 2 --conf-threshold 0.35

.envs/habitat/bin/python scripts/classify_structure_objects.py   --boxes "$box_dir/full_v1_3dbbs_fused.csv"   --output "$stage1_dir/structure_policy.json"   --budget-cny "${PRE_MAP_VLN_QWEN_BUDGET_CNY:-20}" | tee "$run_dir/structure_policy.log"

.envs/habitat/bin/python scripts/build_occusg_grid.py   "$episode_dir" "$grid_dir/${scene_id}_navigation"   --semantic-labels "$semantic_dir/$scene_id.csv" >"$run_dir/navigation_grid.log"

.envs/habitat/bin/python scripts/build_occusg_grid.py   "$episode_dir" "$grid_dir/${scene_id}_structure"   --semantic-labels "$semantic_dir/$scene_id.csv"   --object-boxes "$box_dir/full_v1_3dbbs_fused.csv"   --structure-policy "$stage1_dir/structure_policy.json" >"$run_dir/structure_grid.log"

./scripts/run_occusg.sh "$scene_id" 1.8   "/workspace/benchmark/maps/grids/${scene_id}_structure.npy"   "/workspace/benchmark/maps/grids/${scene_id}_structure.json"   "/workspace/benchmark/maps/occusg/$scene_id/regions.json"   "/workspace/benchmark/maps/occusg/$scene_id/occusg.log"

.envs/habitat/bin/python scripts/fuse_rooms_boxes.py   "$occusg_dir/regions.json" "$box_dir/full_v1_3dbbs_fused.csv"   "$scene_graph_dir/$scene_id.json"   --grid-meta "$grid_dir/${scene_id}_structure.json"

docker compose run --rm falcon python3   /workspace/falcon_ws/src/pre_map_bridge/scripts/inject_progressive_boxes.py   "/workspace/benchmark/bags/${scene_id}_stage1_raw.bag"   "/workspace/benchmark/maps/boxer/$scene_id/full_v1_3dbbs_progressive.csv"   "/workspace/benchmark/bags/${scene_id}_stage1_final.bag"

docker compose run --rm falcon python3 \
  /workspace/falcon_ws/src/pre_map_bridge/scripts/audit_stage1_bag.py \
  --bag "/workspace/benchmark/bags/${scene_id}_stage1_final.bag" \
  --output "/workspace/benchmark/runs/stage1/${scene_id}/bag_audit.json"

python3 - "$status_file" "$scene_id" "$episode_dir" "$scene_graph_dir/$scene_id.json" "$final_bag" <<'PY'
import json,sys
from pathlib import Path
scene_graph=json.loads(Path(sys.argv[4]).read_text(encoding="utf-8"))
exploration=json.loads((Path(sys.argv[1]).parent/"exploration_result.json").read_text(encoding="utf-8"))
bag_audit=json.loads((Path(sys.argv[1]).parent/"bag_audit.json").read_text(encoding="utf-8"))
frames=len(list(Path(sys.argv[3]).glob("frame_*.npz")))
payload={
 "format":"pre_map_vln.benchmark_stage1_status.v1","scene_id":sys.argv[2],"status":"passed",
 "frame_count":frames,"room_count":scene_graph["summary"]["room_count"],
 "object_count":scene_graph["summary"]["object_count"],"final_bag":sys.argv[5],
 "exploration_completion":exploration.get("completion_status"),
 "last_active_frontier_points":bag_audit["active_frontier_points"]["last_point_count"],
 "last_dormant_frontier_points":bag_audit["dormant_frontier_points"]["last_point_count"],
}
Path(sys.argv[1]).write_text(json.dumps(payload,indent=2),encoding="utf-8")
print(json.dumps(payload,ensure_ascii=False))
PY
