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
map_max_z="${RVIZ_MAX_Z:-2.0}"
flight_bag="${RVIZ_FLIGHT_BAG:-}"
flight_topic="${RVIZ_FLIGHT_TOPIC:-/Odometry}"

if [[ -z "$run_id" ]]; then
  echo "Usage: $0 RUN_ID [MIN_OBSERVATIONS] [TASK_ID]" >&2
  echo "Example: $0 Drone_room_03_20260906 1" >&2
  exit 2
fi

run_data="$stage2_dir/data/$run_id"
mapping_bag="${RVIZ_MAPPING_BAG:-$run_data/fastlio_complete/mapping_outputs.bag}"
map_pcd="${RVIZ_MAP_PCD:-$run_data/fastlio_complete/handheld_map_${run_id}_complete.pcd}"
# The canonical visualization is the native FAST-LIO PCD. The older
# global_dense_map is retained for diagnostics and is not silently preferred.
clusters="${RVIZ_CLUSTERS_JSON:-$run_data/localization/target_points_clustered.json}"
raw_targets="${RVIZ_RAW_TARGETS_JSON:-$run_data/localization/target_points_raw.json}"
# Show the same stable geometry used by the semantic scene builder when the
# fused output exists. Fall back to the per-frame CSV for incomplete runs.
if [[ -n "${RVIZ_BOXER_CSV:-}" ]]; then
  boxer_csv="$RVIZ_BOXER_CSV"
elif [[ -s "$run_data/boxer_complete/episode/boxer_3dbbs_fused.csv" ]]; then
  boxer_csv="$run_data/boxer_complete/episode/boxer_3dbbs_fused.csv"
else
  boxer_csv="$run_data/boxer_complete/episode/boxer_3dbbs.csv"
fi
trajectory="${RVIZ_TRAJECTORY:-$run_data/fastlio_complete/trajectory.csv}"
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
if ! [[ "$map_max_z" =~ ^-?[0-9]+([.][0-9]+)?$ ]]; then
  echo "RVIZ_MAX_Z must be a finite numeric world-z limit" >&2
  exit 2
fi
echo "RViz geometry display: world z <= ${map_max_z} m (visualization only)"

flight_args=(-e RVIZ_FLIGHT_BAG= -e RVIZ_FLIGHT_TOPIC="$flight_topic")
if [[ -n "$flight_bag" ]]; then
  [[ "$flight_bag" = /* ]] || flight_bag="$PWD/$flight_bag"
  [[ -s "$flight_bag" ]] || {
    echo "Missing RVIZ_FLIGHT_BAG: $flight_bag" >&2
    exit 2
  }
  flight_args=(
    -e RVIZ_FLIGHT_BAG=/workspace/actual_flight.bag
    -e RVIZ_FLIGHT_TOPIC="$flight_topic"
    -v "$flight_bag:/workspace/actual_flight.bag:ro"
  )
  echo "RViz actual flight: $flight_bag | topic=$flight_topic"
fi

"$root/scripts/prepare_rviz_xauth.sh"

docker_cmd=(docker)
if ! docker info >/dev/null 2>&1; then
  docker_cmd=(sudo -n docker)
fi

mission_rel="${mission#"$root/"}"
voxel_snapshot="${RVIZ_VOXEL_SNAPSHOT:-$run_data/voxel_snapshot}"

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
  "${flight_args[@]}" \
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
  -e RVIZ_MAX_Z="$map_max_z" \
  -e RVIZ_MAP_PCD="/workspace/project/${map_pcd#"$root/"}" \
  -e RVIZ_MAPPING_BAG="/workspace/project/${mapping_bag#"$root/"}" \
  -e RVIZ_CLUSTERS_JSON="/workspace/project/${clusters#"$root/"}" \
  -e RVIZ_RAW_TARGETS_JSON="/workspace/project/${raw_targets#"$root/"}" \
  -e RVIZ_BOXER_CSV="/workspace/project/${boxer_csv#"$root/"}" \
  -e RVIZ_TRAJECTORY="/workspace/project/${trajectory#"$root/"}" \
  -v "$root:/workspace/project" \
  -v "$root/runtime/rviz.Xauthority:/root/.Xauthority:ro" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  "$image" bash -lc '
    set -euo pipefail
    mkdir -p "$XDG_RUNTIME_DIR"
    source /opt/ros/noetic/setup.bash

    python3 /workspace/project/real_fly/stage1_exploration/scripts/export_fastlio_trajectory.py \
      "$RVIZ_MAPPING_BAG" \
      "$(dirname "$RVIZ_TRAJECTORY")"

    export RVIZ_ACTUAL_TRAJECTORY=""
    if [[ -n "$RVIZ_FLIGHT_BAG" ]]; then
      actual_dir=/tmp/real_scene_actual_flight
      python3 /workspace/project/real_fly/stage1_exploration/scripts/export_fastlio_trajectory.py \
        "$RVIZ_FLIGHT_BAG" "$actual_dir" --topic "$RVIZ_FLIGHT_TOPIC"
      export RVIZ_ACTUAL_TRAJECTORY="$actual_dir/trajectory.csv"
    fi

    roscore >/tmp/real_scene_rviz_roscore.log 2>&1 &
    master=$!
    publisher=""
    trap "kill $master $publisher 2>/dev/null || true" EXIT INT TERM
    sleep 2

    publisher_args=( \
      /workspace/project/real_fly/stage2_offline/rviz/publish_drone_room_visualization.py \
      "$RVIZ_MAP_PCD" \
      "$RVIZ_CLUSTERS_JSON" \
      "$RVIZ_RAW_TARGETS_JSON" \
      "$RVIZ_BOXER_CSV" \
      "$RVIZ_TRAJECTORY" \
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
