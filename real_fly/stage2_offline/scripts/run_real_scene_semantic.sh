#!/usr/bin/env bash
# Generic offline Stage-1 bag -> RGB-D/Boxer -> semantic scene graph pipeline.
# It never starts ROS hardware, MAVROS, PX4, or a motion-command publisher.
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  run_real_scene_semantic.sh --run-id ID [options]

Required input:
  real_fly/stage1_exploration/data/ID/raw/sensors.bag

Options:
  --run-id ID                 Input/output run id.
  --input-run-id ID           Existing Stage-1 Bag run id; defaults to --run-id.
  --boxer-mode complete|smoke Complete fused Boxer pass (default: complete).
  --2d-detector grounding_dino|owlv2
                              2-D detector; default is grounding_dino.
  --frame-period SEC          RGB-D keyframe interval (default: 1.0).
  --max-frames N              Maximum exported RGB-D frames (default: 999).
  --depth-source raw|aligned  Depth source; raw is the default and recommended.
  --boxer-python PATH         Boxer Python environment.
  --skip-fastlio              Reuse existing fastlio_complete output.
  --skip-boxer                Reuse existing Boxer output; fail if absent.
  --resume                    Reuse completed stages under an existing output directory.
  --with-planning             Continue from semantic map into offline planning.
  -h, --help                  Show this help.

Every run is isolated below real_fly/stage2_offline/data/ID and is never
allowed to overwrite an existing output directory.
EOF
}

run_id=""
input_run_id=""
boxer_mode="complete"
detector_2d="grounding_dino"
frame_period="1.0"
max_frames="999"
depth_source="raw"
boxer_python=""
skip_fastlio=0
skip_boxer=0
with_planning=0
resume=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --run-id) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; run_id="$2"; shift 2 ;;
    --input-run-id) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; input_run_id="$2"; shift 2 ;;
    --boxer-mode) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; boxer_mode="$2"; shift 2 ;;
    --2d-detector) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; detector_2d="$2"; shift 2 ;;
    --frame-period) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; frame_period="$2"; shift 2 ;;
    --max-frames) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; max_frames="$2"; shift 2 ;;
    --depth-source) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; depth_source="$2"; shift 2 ;;
    --boxer-python) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; boxer_python="$2"; shift 2 ;;
    --skip-fastlio) skip_fastlio=1; shift ;;
    --skip-boxer) skip_boxer=1; shift ;;
    --with-planning) with_planning=1; shift ;;
    --resume) resume=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ "$run_id" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] || { echo "Invalid --run-id: $run_id" >&2; exit 2; }
if [[ -z "$input_run_id" ]]; then input_run_id="$run_id"; fi
[[ "$input_run_id" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] || { echo "Invalid --input-run-id: $input_run_id" >&2; exit 2; }
[[ "$boxer_mode" == complete || "$boxer_mode" == smoke ]] || { echo "--boxer-mode must be complete or smoke" >&2; exit 2; }
[[ "$detector_2d" == grounding_dino || "$detector_2d" == owlv2 ]] || { echo "--2d-detector must be grounding_dino or owlv2" >&2; exit 2; }
[[ "$depth_source" == raw || "$depth_source" == aligned ]] || { echo "--depth-source must be raw or aligned" >&2; exit 2; }
[[ "$frame_period" =~ ^[0-9]+([.][0-9]+)?$ && "$frame_period" != 0 ]] || { echo "Invalid --frame-period" >&2; exit 2; }
[[ "$max_frames" =~ ^[1-9][0-9]*$ ]] || { echo "Invalid --max-frames" >&2; exit 2; }

if [[ "$detector_2d" == grounding_dino ]]; then
  boxer_detector="grounding_dino"
  detector_threshold="0.25"
else
  boxer_detector="owl"
  detector_threshold="0.18"
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
STAGE2_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
ROOT="$(cd -- "$STAGE2_DIR/../.." && pwd)"
STAGE1_DIR="$ROOT/real_fly/stage1_exploration"
RUN_DATA="$STAGE2_DIR/data/$run_id"
BAG="$STAGE1_DIR/data/$input_run_id/raw/sensors.bag"
FASTLIO="$RUN_DATA/fastlio_complete"
FASTLIO_RUN_ID="${run_id}_complete"
LOCAL_MAP="$FASTLIO/handheld_map_${FASTLIO_RUN_ID}.pcd"
GLOBAL_MAP="$FASTLIO/global_dense_map.pcd"
GLOBAL_MAP_REPORT="$FASTLIO/global_dense_map.json"
MAP="$GLOBAL_MAP"
MAPPING_BAG="$FASTLIO/mapping_outputs.bag"
EPISODE="$RUN_DATA/episode"
VOXEL="$RUN_DATA/voxel_snapshot"
BOXER_OUT="$RUN_DATA/boxer_complete"
BOXER_SCENE="$BOXER_OUT/episode"
LOCALIZATION="$RUN_DATA/localization"
SCENE_GRAPH="$RUN_DATA/scene_graph.json"
TASK_GRAPH="$RUN_DATA/task_graph.json"
START_JSON="$RUN_DATA/planning_start.json"
PLANNING_DIR="$RUN_DATA/planning"
RUNTIME_ROOT="$STAGE1_DIR/runtime/fast_lio_${run_id}"
DLS_WS="$STAGE1_DIR/runtime/nx_underlay/dls_ws"
NOETIC_IMAGE="pre-map-vln/real-fly-noetic-offline:local"
BOXER_PY="${boxer_python:-${BOXER_PY:-$ROOT/.envs/boxer/bin/python}}"
CALIBRATION="$ROOT/real_fly/calibration/config/calib_01_lidar_camera.json"
FASTLIO_CONFIG="$STAGE1_DIR/config/fast_lio_mid360_handheld.yaml"
PLANNING_CONFIG="$STAGE2_DIR/config/uav_3d_planning_real.yaml"
CONTAINER_ROOT="/home/nv/SL_WS/PRE_MAP_VLN_real_fly"
CONTAINER_STAGE1="$CONTAINER_ROOT/real_fly/stage1_exploration"
CONTAINER_STAGE2="$CONTAINER_ROOT/real_fly/stage2_offline"
CONTAINER_INPUT_RUN_ID="$input_run_id"

echo "Safety scope: offline only; source Bag is read-only; no flight/control node is started."
[[ -f "$BAG" ]] || { echo "Missing Bag: $BAG" >&2; exit 2; }
if [[ -e "$RUN_DATA" && "$resume" -eq 0 ]]; then
  echo "Refusing to overwrite existing output: $RUN_DATA" >&2
  echo "Use --resume to continue completed stages, or choose a new --run-id." >&2
  exit 2
fi
[[ -x "$BOXER_PY" ]] || { echo "Missing Boxer Python: $BOXER_PY" >&2; exit 2; }
[[ -f "$ROOT/third_party/boxer/run_boxer.py" ]] || { echo "Missing Boxer checkout." >&2; exit 2; }
[[ -f "$CALIBRATION" && -f "$FASTLIO_CONFIG" ]] || { echo "Calibration or FAST-LIO config missing." >&2; exit 2; }
[[ -f "$STAGE1_DIR/runtime/nx_underlay/dls_ws/devel/setup.bash" ]] || {
  echo "Missing isolated Livox underlay: $DLS_WS" >&2; exit 2;
}

if ! rg -q -- '--resume' "$ROOT/third_party/boxer/run_boxer.py"; then
  echo "Boxer checkout lacks the required --resume integration. Refusing to patch the nested repository automatically." >&2
  echo "Apply patches/boxer-real-episode-resume.patch in third_party/boxer first." >&2
  exit 2
fi

DOCKER=(sudo docker run --rm --init --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -v "$ROOT:/home/nv/SL_WS/PRE_MAP_VLN_real_fly" -v "$DLS_WS:/home/nv/dls_ws")

if ! sudo docker image inspect "$NOETIC_IMAGE" >/dev/null 2>&1; then
  sudo docker build -f "$STAGE2_DIR/docker/Dockerfile.noetic-offline" -t "$NOETIC_IMAGE" "$ROOT"
fi

mkdir -p "$RUN_DATA"

if [[ "$resume" -eq 1 && -s "$LOCAL_MAP" && -s "$MAPPING_BAG" ]]; then
  skip_fastlio=1
  echo "Resume: reuse completed FAST-LIO output: $FASTLIO"
fi

if [[ "$skip_fastlio" -eq 0 ]]; then
  FASTLIO_BINARY="$STAGE1_DIR/ros_ws/devel/lib/stage1_fast_lio/fastlio_mapping"
  CONTAINER_RUNTIME_ROOT="$CONTAINER_STAGE1/runtime/fast_lio_${run_id}"
  runtime_ready=0
  if [[ -d "$RUNTIME_ROOT/Log" && -d "$RUNTIME_ROOT/PCD" && -x "$FASTLIO_BINARY" ]] && \
      strings "$FASTLIO_BINARY" | /usr/bin/grep -F "$CONTAINER_RUNTIME_ROOT/" >/dev/null; then
    runtime_ready=1
  fi
  if [[ "$runtime_ready" -eq 0 ]]; then
    echo "Preparing FAST-LIO for isolated runtime root: $RUNTIME_ROOT"
    "${DOCKER[@]}" "$NOETIC_IMAGE" bash -lc \
      "source /opt/ros/noetic/setup.bash && source /home/nv/dls_ws/devel/setup.bash && $CONTAINER_STAGE1/scripts/build_stage1_fast_lio.sh --runtime-root $CONTAINER_RUNTIME_ROOT"
  fi
  "${DOCKER[@]}" --shm-size=4g "$NOETIC_IMAGE" bash -lc \
      "source /opt/ros/noetic/setup.bash && source /home/nv/dls_ws/devel/setup.bash && $CONTAINER_STAGE1/scripts/run_mapping_auto_rates.sh --bag $CONTAINER_STAGE1/data/$CONTAINER_INPUT_RUN_ID/raw/sensors.bag --run-id $FASTLIO_RUN_ID --runtime-root $CONTAINER_STAGE1/runtime/fast_lio_${run_id} --output-dir $CONTAINER_STAGE2/data/$run_id/fastlio_complete"
else
  echo "Reuse requested for FAST-LIO output: $FASTLIO"
fi
[[ -s "$LOCAL_MAP" && -s "$MAPPING_BAG" ]] || { echo "FAST-LIO output missing: $FASTLIO" >&2; exit 2; }

if [[ "$resume" -eq 1 && -s "$GLOBAL_MAP" && -s "$GLOBAL_MAP_REPORT" ]]; then
  echo "Resume: reuse accumulated global point cloud: $GLOBAL_MAP"
else
  [[ ! -e "$GLOBAL_MAP" && ! -e "$GLOBAL_MAP_REPORT" ]] || {
    echo "Partial global point-cloud output exists; use --resume only for a completed pair or choose a new --run-id." >&2
    exit 2
  }
  "${DOCKER[@]}" --shm-size=4g "$NOETIC_IMAGE" bash -lc \
    "source /opt/ros/noetic/setup.bash && python3 $CONTAINER_STAGE2/scripts/accumulate_registered_clouds.py $CONTAINER_STAGE2/data/$run_id/fastlio_complete/mapping_outputs.bag $CONTAINER_STAGE2/data/$run_id/fastlio_complete/global_dense_map.pcd --voxel-size 0.08 --report $CONTAINER_STAGE2/data/$run_id/fastlio_complete/global_dense_map.json"
fi
[[ -s "$GLOBAL_MAP" && -s "$GLOBAL_MAP_REPORT" ]] || { echo "Global point-cloud output missing: $FASTLIO" >&2; exit 2; }

if [[ "$resume" -eq 1 && -f "$EPISODE/manifest.json" ]]; then
  echo "Resume: reuse exported RGB-D episode: $EPISODE"
else
  [[ ! -e "$EPISODE" ]] || { echo "Partial episode exists without manifest; remove or rename it before retrying: $EPISODE" >&2; exit 2; }
  "${DOCKER[@]}" --shm-size=4g "$NOETIC_IMAGE" bash -lc \
    "source /opt/ros/noetic/setup.bash && source /home/nv/dls_ws/devel/setup.bash && python3 $CONTAINER_STAGE2/scripts/export_real_boxer_episode.py $CONTAINER_STAGE1/data/$CONTAINER_INPUT_RUN_ID/raw/sensors.bag $CONTAINER_STAGE2/data/$run_id/fastlio_complete/handheld_map_${FASTLIO_RUN_ID}.pcd $CONTAINER_STAGE2/data/$run_id/episode --mapping-bag $CONTAINER_STAGE2/data/$run_id/fastlio_complete/mapping_outputs.bag --pose-bag $CONTAINER_STAGE2/data/$run_id/fastlio_complete/mapping_outputs.bag --pose-topic /Odometry --frame-period $frame_period --max-frames $max_frames --depth-source $depth_source --lidar-camera-extrinsic $CONTAINER_ROOT/real_fly/calibration/config/calib_01_lidar_camera.json --fastlio-config $CONTAINER_STAGE1/config/fast_lio_mid360_handheld.yaml"
fi

if [[ "$resume" -eq 1 && -f "$VOXEL/metadata.json" ]]; then
  echo "Resume: reuse voxel snapshot: $VOXEL"
else
  [[ ! -e "$VOXEL" ]] || { echo "Partial voxel snapshot exists without metadata; remove or rename it before retrying: $VOXEL" >&2; exit 2; }
  "${DOCKER[@]}" "$NOETIC_IMAGE" bash -lc \
    "source /opt/ros/noetic/setup.bash && source /home/nv/dls_ws/devel/setup.bash && python3 $CONTAINER_STAGE2/scripts/build_real_voxel_snapshot.py $CONTAINER_STAGE2/data/$run_id/fastlio_complete/mapping_outputs.bag $CONTAINER_STAGE2/data/$run_id/fastlio_complete/handheld_map_${FASTLIO_RUN_ID}.pcd $CONTAINER_STAGE2/config/uav_3d_planning_real.yaml $CONTAINER_STAGE2/data/$run_id/voxel_snapshot"
fi

if [[ "$skip_boxer" -eq 1 ]]; then
  echo "Reuse requested for Boxer output: $BOXER_SCENE"
else
  LABELS="$(sed -e 's/#.*$//' -e '/^[[:space:]]*$/d' "$ROOT/config/stage1_indoor_v2.txt" | paste -sd, -)"
  mkdir -p "$BOXER_OUT"
  # The project-specific HabitatLoader lives outside the third-party Boxer
  # checkout; keep it on PYTHONPATH for the detector and helper scripts.
  export PYTHONPATH="$ROOT/boxer_ext:$ROOT/third_party/boxer${PYTHONPATH:+:$PYTHONPATH}"
  if [[ "$boxer_mode" == complete ]]; then
    "$BOXER_PY" "$ROOT/third_party/boxer/run_boxer.py" \
      --input "$EPISODE" --skip_n 1 --max_n 999 --labels="$LABELS" \
      --detector "$boxer_detector" --thresh2d "$detector_threshold" \
      --thresh3d 0.20 --skip_viz --fuse --resume --output_dir "$BOXER_OUT"
  else
    "$BOXER_PY" "$ROOT/third_party/boxer/run_boxer.py" \
      --input "$EPISODE" --skip_n 3 --max_n 999 --labels="$LABELS" \
      --detector "$boxer_detector" --thresh2d "$detector_threshold" \
      --thresh3d 0.20 --skip_viz --fuse --resume --output_dir "$BOXER_OUT"
  fi
fi
[[ -s "$BOXER_SCENE/boxer_3dbbs_fused.csv" || -s "$BOXER_SCENE/boxer_3dbbs.csv" ]] || {
  echo "No Boxer 3-D detections found under $BOXER_SCENE" >&2; exit 2;
}

"$BOXER_PY" "$SCRIPT_DIR/summarize_boxer_output.py" "$BOXER_SCENE"
"$BOXER_PY" "$SCRIPT_DIR/localize_boxer_detections.py" \
  --episode "$EPISODE" --detections-2d "$BOXER_SCENE/owl_2dbbs.csv" \
  --output-dir "$LOCALIZATION" --cluster-radius-m 0.8 --min-cluster-observations 2

"${DOCKER[@]}" "$NOETIC_IMAGE" bash -lc \
  "source /opt/ros/noetic/setup.bash && python3 $CONTAINER_STAGE2/scripts/build_boxer_scene_and_task.py --boxer-dir $CONTAINER_STAGE2/data/$run_id/boxer_complete/episode --voxel-snapshot $CONTAINER_STAGE2/data/$run_id/voxel_snapshot --planning-config $CONTAINER_STAGE2/config/uav_3d_planning_real.yaml --scene-output $CONTAINER_STAGE2/data/$run_id/scene_graph.json --task-output $CONTAINER_STAGE2/data/$run_id/task_graph.json --start-output $CONTAINER_STAGE2/data/$run_id/planning_start.json --prefer-raw"

if [[ "$with_planning" -eq 1 ]]; then
  mkdir -p "$PLANNING_DIR"
  read -r sx sy sz syaw < <("$BOXER_PY" -c 'import json,sys; print(*json.load(open(sys.argv[1]))["start_xyz_yaw"])' "$START_JSON")
  "${DOCKER[@]}" "$NOETIC_IMAGE" python3 "$CONTAINER_ROOT/scripts/plan_stage2_mission.py" \
    --task-graph "$CONTAINER_STAGE2/data/$run_id/task_graph.json" \
    --scene-graph "$CONTAINER_STAGE2/data/$run_id/scene_graph.json" \
    --voxel-snapshot "$CONTAINER_STAGE2/data/$run_id/voxel_snapshot" \
    --planning-config "$CONTAINER_STAGE2/config/uav_3d_planning_real.yaml" \
    --motion-cache "$CONTAINER_STAGE2/data/$run_id/planning/motion_cost_cache.json" \
    --start "$sx" "$sy" "$sz" "$syaw" --output-dir "$CONTAINER_STAGE2/data/$run_id/planning"
fi

echo "Semantic map ready: $SCENE_GRAPH"
echo "Episode: $EPISODE"
echo "Voxel snapshot: $VOXEL"
