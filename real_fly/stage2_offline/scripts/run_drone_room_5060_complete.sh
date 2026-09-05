#!/usr/bin/env bash
# End-to-end, offline-only Drone_room processing for the RTX 5060 host.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STAGE2_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
ROOT="$(cd "$STAGE2_DIR/../.." && pwd)"
STAGE1_DIR="$ROOT/real_fly/stage1_exploration"
DATA="$STAGE2_DIR/data/Drone_room"
BAG="$STAGE1_DIR/data/Drone_room/raw/sensors.bag"
DLS_WS="$STAGE1_DIR/runtime/nx_underlay/dls_ws"
RUNTIME_ROOT="$STAGE1_DIR/runtime/fast_lio_drone_room_5060"
FASTLIO="$DATA/fastlio_complete"
MAP="$FASTLIO/handheld_map_Drone_room_complete.pcd"
MAPPING_BAG="$FASTLIO/mapping_outputs.bag"
EPISODE="$DATA/scene9002_00"
VOXEL="$DATA/voxel_snapshot_complete"
BOXER_OUT="$DATA/boxer_final_5060_complete"
BOXER_SCENE="$BOXER_OUT/scene9002_00"
LOCALIZATION="$DATA/localization"
STAGE2_COMPLETE="$DATA/stage2_complete"
PLANNING="$STAGE2_COMPLETE/planning"
REPORTS="$DATA/reports"
NOETIC_IMAGE="pre-map-vln/real-fly-noetic-offline:local"
BOXER_PY="${BOXER_PY:-$ROOT/.envs/boxer/bin/python}"
DOCKER=(sudo docker run --rm --init --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -v "$ROOT:/home/nv/SL_WS/PRE_MAP_VLN_real_fly" \
  -v "$DLS_WS:/home/nv/dls_ws")

echo "Safety scope: offline only; no MAVROS, PX4Ctrl, setpoint, sensor driver, or flight node is started."
[[ -f "$BAG" ]] || { echo "Missing source bag: $BAG" >&2; exit 2; }
[[ -x "$BOXER_PY" ]] || { echo "Missing Boxer Python environment: $BOXER_PY" >&2; exit 2; }
[[ -f "$ROOT/third_party/boxer/run_boxer.py" ]] || { echo "Missing Boxer checkout." >&2; exit 2; }

# The Boxer checkout is intentionally kept as a third-party tree. Apply the
# tracked project patch only when this checkout has not received the episode
# loader/resume extensions yet.
if ! rg -q -- '--resume' "$ROOT/third_party/boxer/run_boxer.py"; then
  [[ -d "$ROOT/third_party/boxer/.git" ]] || { echo "Boxer checkout has no git metadata; cannot apply the tracked integration patch." >&2; exit 2; }
  git -C "$ROOT/third_party/boxer" apply "$STAGE2_DIR/patches/boxer-real-episode-resume.patch"
fi

if ! sudo docker image inspect "$NOETIC_IMAGE" >/dev/null 2>&1; then
  sudo docker build -f "$STAGE2_DIR/docker/Dockerfile.noetic-offline" -t "$NOETIC_IMAGE" "$ROOT"
fi

# Adopt an existing uniquely generated map before deciding whether remapping is needed.
if [[ ! -s "$MAP" && -s "$MAPPING_BAG" ]]; then
  map_candidate="$(find "$FASTLIO" -maxdepth 1 -type f -name 'handheld_map_Drone_room_complete_*.pcd' -size +0c | sort | tail -n 1)"
  [[ -n "$map_candidate" ]] || { echo "No generated FAST-LIO PCD found in $FASTLIO" >&2; exit 2; }
  cp --reflink=auto "$map_candidate" "$MAP"
fi

if [[ ! -s "$MAPPING_BAG" || ! -s "$MAP" ]]; then
  [[ -f "$DLS_WS/devel/setup.bash" ]] || {
    echo "Missing isolated Livox underlay. Copy/build the NX FAST-LIO and livox_ros_driver2 sources first." >&2
    exit 2
  }
  if [[ ! -x "$STAGE1_DIR/ros_ws/devel/lib/stage1_fast_lio/fastlio_mapping" ]]; then
    "${DOCKER[@]}" "$NOETIC_IMAGE" bash -lc \
      'source /opt/ros/noetic/setup.bash && source /home/nv/dls_ws/devel/setup.bash && /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage1_exploration/scripts/build_stage1_fast_lio.sh --runtime-root /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage1_exploration/runtime/fast_lio_drone_room_5060'
  fi
  "${DOCKER[@]}" --shm-size=4g "$NOETIC_IMAGE" bash -lc \
    'source /opt/ros/noetic/setup.bash && source /home/nv/dls_ws/devel/setup.bash && /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage1_exploration/scripts/run_mapping_auto_rates.sh --bag /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage1_exploration/data/Drone_room/raw/sensors.bag --run-id Drone_room_complete --runtime-root /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage1_exploration/runtime/fast_lio_drone_room_5060 --output-dir /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage2_offline/data/Drone_room/fastlio_complete'
else
  echo "Reuse complete FAST-LIO output: $FASTLIO"
fi

# Keep a stable map filename for downstream tools and reproducible reruns.
if [[ ! -s "$MAP" ]]; then
  map_candidate="$(find "$FASTLIO" -maxdepth 1 -type f -name 'handheld_map_Drone_room_complete_*.pcd' -size +0c | sort | tail -n 1)"
  [[ -n "$map_candidate" ]] || { echo "No generated FAST-LIO PCD found in $FASTLIO" >&2; exit 2; }
  cp --reflink=auto "$map_candidate" "$MAP"
fi

if [[ ! -f "$EPISODE/manifest.json" ]]; then
  "${DOCKER[@]}" --shm-size=4g "$NOETIC_IMAGE" bash -lc \
    'source /opt/ros/noetic/setup.bash && source /home/nv/dls_ws/devel/setup.bash && python3 /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage2_offline/scripts/export_real_boxer_episode.py /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage1_exploration/data/Drone_room/raw/sensors.bag /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage2_offline/data/Drone_room/fastlio_complete/handheld_map_Drone_room_complete.pcd /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage2_offline/data/Drone_room/scene9002_00 --mapping-bag /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage2_offline/data/Drone_room/fastlio_complete/mapping_outputs.bag --frame-period 1.0 --max-frames 999'
else
  echo "Reuse exported RGB-D keyframes: $EPISODE"
fi

if [[ ! -f "$VOXEL/report.json" ]]; then
  "${DOCKER[@]}" "$NOETIC_IMAGE" bash -lc \
    'source /opt/ros/noetic/setup.bash && source /home/nv/dls_ws/devel/setup.bash && python3 /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage2_offline/scripts/build_real_voxel_snapshot.py /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage2_offline/data/Drone_room/fastlio_complete/mapping_outputs.bag /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage2_offline/data/Drone_room/fastlio_complete/handheld_map_Drone_room_complete.pcd /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage2_offline/config/uav_3d_planning_real.yaml /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage2_offline/data/Drone_room/voxel_snapshot_complete'
else
  echo "Reuse voxel snapshot: $VOXEL"
fi

boxer_complete="$($BOXER_PY -c 'import json,sys; p=sys.argv[1];
try: print(json.load(open(p)).get("status", "") == "completed")
except (FileNotFoundError, json.JSONDecodeError): print(False)' "$BOXER_SCENE/processing_manifest.json")"
if [[ "$boxer_complete" != "True" ]]; then
  BOXER_LABELS="$(sed -e 's/#.*$//' -e '/^[[:space:]]*$/d' "$ROOT/config/stage1_indoor_v2.txt" | paste -sd, -)"
  export PYTHONPATH="$ROOT/boxer_ext${PYTHONPATH:+:$PYTHONPATH}"
  "$BOXER_PY" "$ROOT/third_party/boxer/run_boxer.py" \
    --input "$EPISODE" --skip_n 1 --max_n 999 --labels="$BOXER_LABELS" \
    --thresh2d 0.18 --thresh3d 0.20 --skip_viz --fuse --resume --output_dir "$BOXER_OUT"
else
  echo "Reuse completed CUDA Boxer pass: $BOXER_SCENE"
fi

"$BOXER_PY" "$SCRIPT_DIR/summarize_boxer_output.py" "$BOXER_SCENE"
"$BOXER_PY" "$SCRIPT_DIR/localize_boxer_detections.py" \
  --episode "$EPISODE" --detections-2d "$BOXER_SCENE/owl_2dbbs.csv" \
  --output-dir "$LOCALIZATION" --cluster-radius-m 0.8 --min-cluster-observations 2

mkdir -p "$STAGE2_COMPLETE" "$PLANNING" "$REPORTS"
"${DOCKER[@]}" "$NOETIC_IMAGE" bash -lc \
  'source /opt/ros/noetic/setup.bash && python3 /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage2_offline/scripts/build_boxer_scene_and_task.py --boxer-dir /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage2_offline/data/Drone_room/boxer_final_5060_complete/scene9002_00 --voxel-snapshot /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage2_offline/data/Drone_room/voxel_snapshot_complete --planning-config /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage2_offline/config/uav_3d_planning_real.yaml --scene-output /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage2_offline/data/Drone_room/stage2_complete/scene_graph.json --task-output /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage2_offline/data/Drone_room/stage2_complete/task_graph.json --start-output /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage2_offline/data/Drone_room/stage2_complete/planning_start.json --prefer-raw --target-label box'

read -r SX SY SZ SYAW < <("$BOXER_PY" -c 'import json,sys; print(*json.load(open(sys.argv[1]))["start_xyz_yaw"])' "$STAGE2_COMPLETE/planning_start.json")
"${DOCKER[@]}" "$NOETIC_IMAGE" python3 /home/nv/SL_WS/PRE_MAP_VLN_real_fly/scripts/plan_stage2_mission.py \
  --task-graph /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage2_offline/data/Drone_room/stage2_complete/task_graph.json \
  --scene-graph /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage2_offline/data/Drone_room/stage2_complete/scene_graph.json \
  --voxel-snapshot /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage2_offline/data/Drone_room/voxel_snapshot_complete \
  --planning-config /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage2_offline/config/uav_3d_planning_real.yaml \
  --motion-cache /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage2_offline/data/Drone_room/stage2_complete/planning/motion_cost_cache.json \
  --start "$SX" "$SY" "$SZ" "$SYAW" \
  --output-dir /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage2_offline/data/Drone_room/stage2_complete/planning

"${DOCKER[@]}" "$NOETIC_IMAGE" bash -lc \
  'source /opt/ros/noetic/setup.bash && python3 /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage2_offline/scripts/validate_drone_room_stage2.py --bag /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage1_exploration/data/Drone_room/raw/sensors.bag --mapping-bag /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage2_offline/data/Drone_room/fastlio_complete/mapping_outputs.bag --episode /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage2_offline/data/Drone_room/scene9002_00 --boxer-dir /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage2_offline/data/Drone_room/boxer_final_5060_complete/scene9002_00 --localization-summary /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage2_offline/data/Drone_room/localization/localization_summary.json --voxel-snapshot /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage2_offline/data/Drone_room/voxel_snapshot_complete --planning-config /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage2_offline/config/uav_3d_planning_real.yaml --scene-graph /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage2_offline/data/Drone_room/stage2_complete/scene_graph.json --task-graph /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage2_offline/data/Drone_room/stage2_complete/task_graph.json --planning-dir /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage2_offline/data/Drone_room/stage2_complete/planning --output /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage2_offline/data/Drone_room/reports/stage2_validation_complete.json'
"${DOCKER[@]}" "$NOETIC_IMAGE" bash -lc \
  'source /opt/ros/noetic/setup.bash && source /home/nv/dls_ws/devel/setup.bash && python3 /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage2_offline/scripts/audit_real_stage2_quality.py --bag /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage1_exploration/data/Drone_room/raw/sensors.bag --mapping-bag /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage2_offline/data/Drone_room/fastlio_complete/mapping_outputs.bag --pcd /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage2_offline/data/Drone_room/fastlio_complete/handheld_map_Drone_room_complete.pcd --episode /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage2_offline/data/Drone_room/scene9002_00 --voxel-snapshot /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage2_offline/data/Drone_room/voxel_snapshot_complete --planning-config /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage2_offline/config/uav_3d_planning_real.yaml --mission /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage2_offline/data/Drone_room/stage2_complete/planning/mission_plan.json --output /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage2_offline/data/Drone_room/reports/stage2_quality_complete.json'

echo "Drone_room complete offline processing finished: $DATA"
