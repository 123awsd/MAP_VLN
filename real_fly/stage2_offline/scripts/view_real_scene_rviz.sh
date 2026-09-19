#!/usr/bin/env bash
set -euo pipefail

# View one generic real-fly semantic-map run in RViz.
# This is offline-only: it reads the recorded bag and generated products;
# it does not start FAST-LIO, Habitat, or any flight/control node.

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
stage2_dir="$(cd "$script_dir/.." && pwd)"
root="$(cd "$stage2_dir/../.." && pwd)"
image="pre-map-vln/falcon-noetic:local"

run_id="${1:-}"
min_observations="${2:-1}"
task_id="${3:-}"

if [[ -z "$run_id" ]]; then
  echo "Usage: $0 RUN_ID [MIN_OBSERVATIONS] [TASK_ID]" >&2
  echo "Example: $0 Drone_room_03_20260906 1" >&2
  exit 2
fi

run_data="$stage2_dir/data/$run_id"
mapping_bag="$run_data/fastlio_complete/mapping_outputs.bag"
map_pcd="${RVIZ_MAP_PCD:-$run_data/fastlio_complete/handheld_map_${run_id}_complete.pcd}"
# The canonical visualization is the native FAST-LIO PCD. The older
# global_dense_map is retained for diagnostics and is not silently preferred.
clusters="$run_data/localization/target_points_clustered.json"
raw_targets="$run_data/localization/target_points_raw.json"
boxer_csv="$run_data/boxer_complete/episode/boxer_3dbbs.csv"
trajectory="$run_data/fastlio_complete/trajectory.csv"
mission="$run_data/planning/mission_plan.json"
candidates_json=""
if [[ -n "$task_id" ]]; then
  [[ "$task_id" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] || {
    echo "Invalid TASK_ID: $task_id" >&2
    exit 2
  }
  mission="$run_data/tasks/$task_id/planning/mission_plan.json"
  [[ -f "$mission" ]] || { echo "Missing task mission: $mission" >&2; exit 2; }
  candidates_json="$run_data/tasks/$task_id/planning/candidates.json"
  [[ -f "$candidates_json" ]] || { echo "Missing task candidates: $candidates_json" >&2; exit 2; }
  final_dir="$root/real_fly/stage2_runtime/missions/$run_id/$task_id"
  if [[ -f "$final_dir/final_minco_manifest.json" ]]; then
    python3 "$root/real_fly/stage2_runtime/scripts/saved_minco_artifact.py" verify \
      --directory "$final_dir" \
      --map "$run_data/fastlio_complete/handheld_map_${run_id}_complete.pcd"
    mission="$final_dir/final_minco_preview.json"
    echo "Preview source: final saved MINCO coefficients (same artifact as NX)"
  else
    echo "No certified final MINCO exists; showing Stage2 geometric preview only."
  fi
fi
# Optional diagnostic override. This is useful for comparing the raw A* route
# against the validated/smoothed mission without changing any planning output.
if [[ -n "${RVIZ_MISSION_JSON:-}" ]]; then
  mission="$RVIZ_MISSION_JSON"
  [[ "$mission" = /* ]] || mission="$root/$mission"
  [[ -f "$mission" ]] || { echo "Missing RVIZ_MISSION_JSON: $mission" >&2; exit 2; }
fi
if [[ ! -f "$mission" ]]; then
  # A semantic-only run has no planned mission yet; planning_start.json is
  # still a valid JSON input and simply leaves the planned-path display empty.
  mission="$run_data/planning_start.json"
fi

for path in "$map_pcd" "$clusters" "$raw_targets" "$boxer_csv" "$mapping_bag"; do
  [[ -f "$path" ]] || { echo "Missing visualization input: $path" >&2; exit 2; }
done

if ! [[ "$min_observations" =~ ^[0-9]+$ ]]; then
  echo "MIN_OBSERVATIONS must be a non-negative integer" >&2
  exit 2
fi

"$root/scripts/prepare_rviz_xauth.sh"

docker_cmd=(docker)
if ! docker info >/dev/null 2>&1; then
  docker_cmd=(sudo -n docker)
fi

mission_rel="${mission#"$root/"}"
voxel_snapshot="$run_data/voxel_snapshot"

# CDI supplies the host-matched NVIDIA graphics libraries and devices only
# to this viewer. Keep CPU rendering available as an explicit compatibility mode.
render_args=()
case "${RVIZ_RENDERER:-nvidia}" in
  nvidia)
    render_args=(--device nvidia.com/gpu=all -e __GLX_VENDOR_LIBRARY_NAME=nvidia)
    echo "RViz renderer: NVIDIA GPU (CDI)"
    ;;
  softpipe)
    render_args=(-e LIBGL_ALWAYS_SOFTWARE=1 -e GALLIUM_DRIVER=softpipe)
    echo "RViz renderer: CPU softpipe (slow compatibility mode)"
    ;;
  *) echo "RVIZ_RENDERER must be nvidia or softpipe" >&2; exit 2 ;;
esac

exec "${docker_cmd[@]}" run --rm --init --net host \
  "${render_args[@]}" \
  -e DISPLAY="${DISPLAY:-:0}" \
  -e XAUTHORITY=/root/.Xauthority \
  -e XDG_RUNTIME_DIR=/tmp/runtime-root \
  -e DISABLE_ROS1_EOL_WARNINGS=1 \
  -e QT_X11_NO_MITSHM=1 \
  -e RVIZ_RUN_ID="$run_id" \
  -e RVIZ_MISSION="/workspace/project/$mission_rel" \
  -e RVIZ_CANDIDATES_JSON="/workspace/project/${candidates_json#"$root/"}" \
  -e RVIZ_VOXEL_SNAPSHOT="/workspace/project/${voxel_snapshot#"$root/"}" \
  -e RVIZ_MIN_OBSERVATIONS="$min_observations" \
  -e RVIZ_MAP_PCD="/workspace/project/${map_pcd#"$root/"}" \
  -v "$root:/workspace/project" \
  -v "$root/runtime/rviz.Xauthority:/root/.Xauthority:ro" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  "$image" bash -lc '
    set -euo pipefail
    mkdir -p "$XDG_RUNTIME_DIR"
    source /opt/ros/noetic/setup.bash

    python3 /workspace/project/real_fly/stage1_exploration/scripts/export_fastlio_trajectory.py \
      "/workspace/project/real_fly/stage2_offline/data/$RVIZ_RUN_ID/fastlio_complete/mapping_outputs.bag" \
      "/workspace/project/real_fly/stage2_offline/data/$RVIZ_RUN_ID/fastlio_complete"

    roscore >/tmp/real_scene_rviz_roscore.log 2>&1 &
    master=$!
    publisher=""
    trap "kill $master $publisher 2>/dev/null || true" EXIT INT TERM
    sleep 2

    publisher_args=( \
      /workspace/project/real_fly/stage2_offline/rviz/publish_drone_room_visualization.py \
      "$RVIZ_MAP_PCD" \
      "/workspace/project/real_fly/stage2_offline/data/$RVIZ_RUN_ID/localization/target_points_clustered.json" \
      "/workspace/project/real_fly/stage2_offline/data/$RVIZ_RUN_ID/localization/target_points_raw.json" \
      "/workspace/project/real_fly/stage2_offline/data/$RVIZ_RUN_ID/boxer_complete/episode/boxer_3dbbs.csv" \
      "/workspace/project/real_fly/stage2_offline/data/$RVIZ_RUN_ID/fastlio_complete/trajectory.csv" \
      "$RVIZ_MISSION" \
      "$RVIZ_MIN_OBSERVATIONS" )
    if [[ -s "$RVIZ_VOXEL_SNAPSHOT/metadata.json" && -s "$RVIZ_VOXEL_SNAPSHOT/voxel_map.npz" ]]; then
      publisher_args+=("$RVIZ_VOXEL_SNAPSHOT")
    fi
    python3 "${publisher_args[@]}" \
      >/tmp/real_scene_rviz_publisher.log 2>&1 &
    publisher=$!

    sleep 3
    rviz -d /workspace/project/real_fly/stage2_offline/rviz/drone_room.rviz
  '
