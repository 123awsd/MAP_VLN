#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
stage2_dir="$(cd "$script_dir/.." && pwd)"
root="$(cd "$stage2_dir/../.." && pwd)"
image="pre-map-vln/falcon-noetic:local"
mode="${1:-reliable}"
dataset="${2:-Drone_room}"
case "$mode" in
  reliable) min_observations=2 ;;
  all) min_observations=1 ;;
  *) echo "Usage: $0 [reliable|all] [Drone_room|Drone_room_02]" >&2; exit 2 ;;
esac
data="$stage2_dir/data/$dataset"

if [[ "$dataset" == "Drone_room_02" ]]; then
  localization="$data/instance_fusion_calib01_rawdepth_rgbclock40ms"
  boxer_scene="$data/boxer_calib01_rawdepth_rgbclock40ms/scene9002_00_calib01_rawdepth_rgbclock40ms"
  mission="$data/stage2_balanced_calib01_rgbclock40ms/planning/mission_plan.json"
  map_pcd="$data/fastlio_complete/handheld_map_Drone_room_02_complete.pcd"
  echo "Using Drone_room_02 raw-depth offline alignment and balanced fusion"
elif [[ -f "$data/scene9002_00_calib01/manifest.json" ]]; then
  if [[ -f "$data/instance_fusion_calib01/fusion_report.json" ]]; then
    localization="$data/instance_fusion_calib01"
    echo "Using balanced semantic instance fusion"
  else
    localization="$data/localization_calib01"
  fi
  boxer_scene="$data/boxer_final_5060_calib01/scene9002_00_calib01"
  if [[ -f "$data/stage2_balanced_calib01/planning/mission_plan.json" ]]; then
    mission="$data/stage2_balanced_calib01/planning/mission_plan.json"
  else
    mission="$data/stage2_calib01/planning/mission_plan.json"
  fi
  echo "Using calibrated result set: calib_01"
  map_pcd="$data/fastlio_complete/handheld_map_Drone_room_complete.pcd"
else
  localization="$data/localization"
  boxer_scene="$data/boxer_final_5060_complete/scene9002_00"
  mission="$data/stage2_complete/planning/mission_plan.json"
  map_pcd="$data/fastlio_complete/handheld_map_Drone_room_complete.pcd"
fi
trajectory="$data/fastlio_complete/trajectory.csv"

for path in "$map_pcd" \
            "$localization/target_points_clustered.json" \
            "$localization/target_points_raw.json" \
            "$boxer_scene/boxer_3dbbs.csv" \
            "$trajectory" \
            "$mission"; do
  [[ -f "$path" ]] || { echo "Missing visualization input: $path" >&2; exit 2; }
done

"$root/scripts/prepare_rviz_xauth.sh"
docker_cmd=(docker)
if ! docker info >/dev/null 2>&1; then docker_cmd=(sudo -n docker); fi
exec "${docker_cmd[@]}" run --rm --init --net host \
  -e DISPLAY="${DISPLAY:-:0}" -e XAUTHORITY=/root/.Xauthority \
  -e RVIZ_MIN_OBSERVATIONS="$min_observations" \
  -e RVIZ_LOCALIZATION="${localization#"$root"/}" \
  -e RVIZ_BOXER_SCENE="${boxer_scene#"$root"/}" \
  -e RVIZ_MISSION="${mission#"$root"/}" \
  -e RVIZ_MAP_PCD="${map_pcd#"$root"/}" \
  -e RVIZ_TRAJECTORY="${trajectory#"$root"/}" \
  -v "$root:/workspace/project:ro" \
  -v "$root/runtime/rviz.Xauthority:/root/.Xauthority:ro" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  "$image" bash -lc '
    set -e
    source /opt/ros/noetic/setup.bash
    publisher=""
    roscore >/tmp/drone_room_roscore.log 2>&1 & master=$!
    trap "kill $master $publisher 2>/dev/null || true" EXIT INT TERM
    sleep 2
    python3 /workspace/project/real_fly/stage2_offline/rviz/publish_drone_room_visualization.py \
      "/workspace/project/$RVIZ_MAP_PCD" \
      "/workspace/project/$RVIZ_LOCALIZATION/target_points_clustered.json" \
      "/workspace/project/$RVIZ_LOCALIZATION/target_points_raw.json" \
      "/workspace/project/$RVIZ_BOXER_SCENE/boxer_3dbbs.csv" \
      "/workspace/project/$RVIZ_TRAJECTORY" \
      "/workspace/project/$RVIZ_MISSION" \
      "$RVIZ_MIN_OBSERVATIONS" \
      >/tmp/drone_room_visualization.log 2>&1 & publisher=$!
    sleep 3
    rviz -d /workspace/project/real_fly/stage2_offline/rviz/drone_room.rviz
  '
