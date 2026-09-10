#!/usr/bin/env bash
# Generate a motion-only mission from stable disarmed localization on the NX.
# This script subscribes only while capturing pose and never publishes a ROS topic.
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  plan_from_current_pose.sh RUN_ID TASK_ID --instruction TEXT

Options:
  --px4ctrl-config FILE  Config supplying relative takeoff_height.
  --qwen-key-file FILE   DashScope key file used only for instruction parsing.
  --budget-cny VALUE     Qwen cost ceiling (default: 20).
  --no-cache             Do not reuse a cached Qwen task graph.
  --max-candidates N     Candidates retained per target (default: 6).
  -h, --help             Show this help.

The FCU must be connected and disarmed, and global world-frame localization must
already be stable. No planner goal, control command, arm, or takeoff is sent.
EOF
}

[[ $# -ge 1 ]] || { usage >&2; exit 2; }
if [[ "$1" == "-h" || "$1" == "--help" ]]; then usage; exit 0; fi
[[ $# -ge 4 ]] || { usage >&2; exit 2; }
run_id="$1"
task_id="$2"
shift 2
instruction=""
px4ctrl_config="/home/nv/dls_ws/src/control/px4ctrl/config/ctrl_param_fpv.yaml"
qwen_key_file="${PRE_MAP_VLN_QWEN_KEY_FILE:-$HOME/.config/pre_map_vln/dashscope_api_key}"
budget_cny="20"
no_cache=0
max_candidates="6"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --instruction) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; instruction="$2"; shift 2 ;;
    --px4ctrl-config) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; px4ctrl_config="$2"; shift 2 ;;
    --qwen-key-file) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; qwen_key_file="$2"; shift 2 ;;
    --budget-cny) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; budget_cny="$2"; shift 2 ;;
    --no-cache) no_cache=1; shift ;;
    --max-candidates) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; max_candidates="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done
[[ "$run_id" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] || { echo "Invalid RUN_ID: $run_id" >&2; exit 2; }
[[ "$task_id" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] || { echo "Invalid TASK_ID: $task_id" >&2; exit 2; }
[[ -n "$instruction" ]] || { echo "Missing required --instruction." >&2; exit 2; }
[[ "$budget_cny" =~ ^[0-9]+([.][0-9]+)?$ ]] || { echo "Invalid --budget-cny." >&2; exit 2; }
[[ "$max_candidates" =~ ^[1-9][0-9]*$ ]] || { echo "Invalid --max-candidates." >&2; exit 2; }

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
runtime_root="$(cd -- "$script_dir/.." && pwd)"
root="$(cd -- "$runtime_root/../.." && pwd)"
package="$runtime_root/map_packages/$run_id"
output="$runtime_root/missions/$run_id/$task_id"
map="/home/nv/dls_ws/${run_id}.pcd"
scene="$package/approved_scene_graph.json"
voxel="$package/voxel_snapshot"
config="$package/uav_3d_planning_real.yaml"
manifest="$package/manifest.json"

for path in "$manifest" "$scene" "$voxel/metadata.json" "$voxel/voxel_map.npz" \
  "$config" "$map" "$px4ctrl_config"; do
  [[ -s "$path" ]] || { echo "Missing runtime-planning input: $path" >&2; exit 2; }
done
[[ ! -e "$output" ]] || { echo "Refusing to overwrite existing task: $output" >&2; exit 2; }

expected_sha="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["map_sha256"])' "$manifest")"
actual_sha="$(sha256sum "$map" | cut -d' ' -f1)"
[[ "$actual_sha" == "$expected_sha" ]] || {
  echo "Map SHA256 mismatch; refusing to mix RUN_ID data." >&2
  exit 2
}
python3 - "$manifest" "$package" <<'PY'
import hashlib
import json
import pathlib
import sys

manifest = json.load(open(sys.argv[1], encoding="utf-8"))
package = pathlib.Path(sys.argv[2])
if manifest.get("format") != "pre_map_vln.nx_planning_package.v1":
    raise SystemExit("unsupported planning package")
for name, expected in manifest.get("files", {}).items():
    path = package / name
    if not path.is_file():
        raise SystemExit(f"missing packaged file: {path}")
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if digest != expected:
        raise SystemExit(f"packaged file SHA256 mismatch: {name}")
PY

unset _CATKIN_SETUP_DIR || true
set +u
source /opt/ros/noetic/setup.bash
source /home/nv/dls_ws/devel/setup.bash
set -u

mkdir -p "$(dirname "$output")" "$output/planning"
planning_complete=0
cleanup_partial() {
  local rc=$?
  trap - EXIT
  if [[ "$planning_complete" -eq 0 && -d "$output" ]]; then
    local failed="${output}.failed.$(date +%Y%m%d_%H%M%S)"
    mv "$output" "$failed"
    echo "Incomplete planning output preserved for diagnosis: $failed" >&2
  fi
  exit "$rc"
}
trap cleanup_partial EXIT
start_json="$output/planning_start_runtime.json"
task_graph="$output/task_graph.json"
raw_mission="$output/planning/mission_plan_astar.json"
mission="$output/planning/mission_plan.json"
audit="$output/mission_audit.json"
bundle="$output/execution_bundle.json"

echo "Safety scope: Qwen task parsing, read-only localization capture and offline planning; no ROS output is published."
[[ -s "$qwen_key_file" ]] || {
  echo "Missing Qwen key file: $qwen_key_file" >&2
  echo "Install it once with mode 600, or pass --qwen-key-file FILE." >&2
  exit 2
}
parser_args=(
  "$root/scripts/parse_stage2_instruction.py" "$instruction"
  --scene-graph "$scene" --output "$task_graph"
  --budget-cny "$budget_cny" --api-key-file "$qwen_key_file"
)
if [[ "$no_cache" -eq 1 ]]; then parser_args+=(--no-cache); fi
python3 "${parser_args[@]}"
python3 "$script_dir/capture_planning_start.py" \
  --output "$start_json" --px4ctrl-config "$px4ctrl_config"
read -r sx sy sz syaw < <(
  python3 -c 'import json,sys; print(*json.load(open(sys.argv[1]))["planned_hover_xyz_yaw"])' "$start_json"
)

python3 "$root/scripts/plan_stage2_mission.py" \
  --task-graph "$task_graph" --scene-graph "$scene" \
  --voxel-snapshot "$voxel" --planning-config "$config" \
  --motion-cache "$output/planning/motion_cost_cache.json" \
  --start "$sx" "$sy" "$sz" "$syaw" --max-candidates "$max_candidates" \
  --output-dir "$output/planning"
mv "$output/planning/mission_plan.json" "$raw_mission"

python3 "$root/real_fly/stage2_offline/scripts/finalize_real_mission.py" \
  --mission "$raw_mission" --task-graph "$task_graph" \
  --voxel-snapshot "$voxel" --planning-config "$config" \
  --start-source explicit_approved_pose \
  --output-mission "$mission" --output-audit "$audit"
python3 "$script_dir/prepare_execution_bundle.py" \
  --mission "$mission" --map "$map" --voxel-metadata "$voxel/metadata.json" \
  --output "$bundle"

planning_complete=1
echo "NX mission prepared: $output"
echo "Review only: $script_dir/view_runtime_plan_rviz.sh $run_id $task_id"
