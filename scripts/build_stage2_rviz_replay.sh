#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
result_dir="$(realpath "${1:?usage: build_stage2_rviz_replay.sh RESULT_DIR SOURCE_STAGE1_BAG [BAG_NAME] [TASK_GRAPH]}")"
source_bag="$(realpath "${2:?usage: build_stage2_rviz_replay.sh RESULT_DIR SOURCE_STAGE1_BAG [BAG_NAME] [TASK_GRAPH]}")"
bag_name="${3:-$(basename "$result_dir")_rviz}"

to_shared() {
  case "$1" in
    "$root_dir"/outputs/*) echo "/workspace/shared/outputs/${1#"$root_dir"/outputs/}" ;;
    *) echo "path must be under $root_dir/outputs: $1" >&2; return 1 ;;
  esac
}

execution_dir="$result_dir/execution"
[[ -f "$execution_dir/habitat_execution.json" ]] || execution_dir="$result_dir"
execution="$execution_dir/habitat_execution.json"
candidates="$execution_dir/candidates_with_recovery.json"
scene_graph="$execution_dir/updated_scene_graph.json"
frames="$execution_dir/frames"
task_graph="${4:-$result_dir/task_graph.json}"
task_graph="$(realpath "$task_graph")"
case "$task_graph" in
  "$root_dir"/outputs/*) ;;
  *)
    staged_task_graph="$result_dir/replay_task_graph.json"
    cp "$task_graph" "$staged_task_graph"
    task_graph="$staged_task_graph"
    ;;
esac
for path in "$execution" "$candidates" "$scene_graph" "$task_graph" "$source_bag"; do
  [[ -e "$path" ]] || { echo "missing replay input: $path" >&2; exit 1; }
done

cd "$root_dir"
docker compose run --rm falcon python3 /workspace/falcon_ws/src/pre_map_bridge/scripts/build_stage2_bag.py \
  --source-bag "$(to_shared "$source_bag")" \
  --execution "$(to_shared "$execution")" \
  --task-graph "$(to_shared "$task_graph")" \
  --candidates "$(to_shared "$candidates")" \
  --scene-graph "$(to_shared "$scene_graph")" \
  --frames-dir "$(to_shared "$frames")" \
  --output-bag "/workspace/shared/outputs/bags/${bag_name}.bag" \
  --hz 5
echo "built=$root_dir/outputs/bags/${bag_name}.bag"
