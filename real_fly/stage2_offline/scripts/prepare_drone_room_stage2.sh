#!/usr/bin/env bash
# Offline-only pipeline. It never starts ROS, MAVROS, a flight controller, or a command publisher.
set -euo pipefail

COMPLETE_BOXER=0
if [[ "${1:-}" == "--complete-boxer" ]]; then
  COMPLETE_BOXER=1
  shift
fi
if [[ $# -ne 0 ]]; then
  echo "Usage: $0 [--complete-boxer]" >&2
  exit 2
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
STAGE2_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROOT="$(cd -- "${STAGE2_DIR}/../.." && pwd)"
PY="${STAGE2_DIR}/.envs/boxer-jp5/bin/python"
BAG="${ROOT}/real_fly/stage1_exploration/data/Drone_room/raw/sensors.bag"
MAP_PCD="${ROOT}/real_fly/stage1_exploration/runtime/live_fast_lio/PCD/handheld_20260905_003121.pcd"
EPISODE="${STAGE2_DIR}/data/Drone_room/scene9002_00"
VOXEL="${STAGE2_DIR}/data/Drone_room/voxel_snapshot"
BOXER_ROOT="${STAGE2_DIR}/third_party/boxer"
BOXER_OUT="${STAGE2_DIR}/data/Drone_room/boxer_final_v2"
BOXER_SCENE="${BOXER_OUT}/scene9002_00"
PLANNING_CONFIG="${STAGE2_DIR}/config/uav_3d_planning_real.yaml"
SCENE_GRAPH="${STAGE2_DIR}/data/Drone_room/scene_graph.json"
TASK_GRAPH="${STAGE2_DIR}/config/drone_room_task.json"
START_JSON="${STAGE2_DIR}/data/Drone_room/planning_start.json"
PLANNING_DIR="${STAGE2_DIR}/data/Drone_room/planning"
VALIDATION="${STAGE2_DIR}/data/Drone_room/reports/stage2_validation.json"

echo "Safety scope: offline processing only; source bag is read-only; no flight-control processes are started."
for required in "${PY}" "${BAG}" "${MAP_PCD}" "${BOXER_ROOT}/run_boxer.py"; do
  if [[ ! -e "${required}" ]]; then
    echo "Missing required input/environment: ${required}" >&2
    exit 2
  fi
done

if [[ ! -f "${EPISODE}/manifest.json" ]]; then
  "${PY}" "${SCRIPT_DIR}/export_real_boxer_episode.py" \
    "${BAG}" "${MAP_PCD}" "${EPISODE}" --frame-period 1.0 --max-frames 180
else
  echo "Reuse exported RGB-D episode: ${EPISODE}"
fi

if [[ ! -f "${VOXEL}/metadata.json" ]]; then
  "${PY}" "${SCRIPT_DIR}/build_real_voxel_snapshot.py" \
    "${BAG}" "${MAP_PCD}" "${PLANNING_CONFIG}" "${VOXEL}"
else
  echo "Reuse conservative voxel snapshot: ${VOXEL}"
fi

if [[ -s "${BOXER_SCENE}/boxer_3dbbs_fused.csv" ]]; then
  echo "Reuse fused Boxer detections: ${BOXER_SCENE}/boxer_3dbbs_fused.csv"
elif [[ -s "${BOXER_SCENE}/boxer_3dbbs.csv" && "${COMPLETE_BOXER}" -eq 0 ]]; then
  echo "Smoke mode: reuse partial unfused Boxer detections: ${BOXER_SCENE}/boxer_3dbbs.csv"
  echo "Use --complete-boxer only when a complete semantic pass is required."
else
  LABELS="$(sed -e 's/#.*$//' -e '/^[[:space:]]*$/d' "${ROOT}/config/stage1_indoor_v2.txt" | paste -sd, -)"
  echo "Run Boxer offline inference (CPU fallback on this NX image; this can take about 40 minutes)."
  (
    cd "${BOXER_ROOT}"
    OPENBLAS_CORETYPE=ARMV8 OMP_NUM_THREADS=6 MKL_NUM_THREADS=6 \
      "${PY}" run_boxer.py --input "${EPISODE}" --skip_n 3 --max_n 999 \
      --labels="${LABELS}" --thresh2d 0.18 --thresh3d 0.20 \
      --skip_viz --force_cpu --fuse --output_dir "${BOXER_OUT}"
  )
  if [[ ! -s "${BOXER_SCENE}/boxer_3dbbs_fused.csv" ]]; then
    echo "Default fusion produced no stable instance; retry with two-view support and 0.30 confidence."
    (
      cd "${BOXER_ROOT}"
      OPENBLAS_CORETYPE=ARMV8 OMP_NUM_THREADS=6 MKL_NUM_THREADS=6 \
        "${PY}" utils/fuse_3d_boxes.py \
        --input "${BOXER_SCENE}/boxer_3dbbs.csv" \
        --output "${BOXER_SCENE}/boxer_3dbbs_fused.csv" \
        --min_detections 2 --conf_threshold 0.30
    )
  fi
fi

"${PY}" "${SCRIPT_DIR}/build_boxer_scene_and_task.py" \
  --boxer-dir "${BOXER_SCENE}" --voxel-snapshot "${VOXEL}" \
  --planning-config "${PLANNING_CONFIG}" --scene-output "${SCENE_GRAPH}" \
  --task-output "${TASK_GRAPH}" --start-output "${START_JSON}"

read -r SX SY SZ SYAW < <("${PY}" -c \
  'import json,sys; print(*json.load(open(sys.argv[1]))["start_xyz_yaw"])' "${START_JSON}")
mkdir -p "${PLANNING_DIR}"
"${PY}" "${ROOT}/scripts/plan_stage2_mission.py" \
  --task-graph "${TASK_GRAPH}" --scene-graph "${SCENE_GRAPH}" \
  --voxel-snapshot "${VOXEL}" --planning-config "${PLANNING_CONFIG}" \
  --motion-cache "${PLANNING_DIR}/motion_cost_cache.json" \
  --start "${SX}" "${SY}" "${SZ}" "${SYAW}" --output-dir "${PLANNING_DIR}"

"${PY}" "${SCRIPT_DIR}/validate_drone_room_stage2.py" \
  --bag "${BAG}" --episode "${EPISODE}" --boxer-dir "${BOXER_SCENE}" \
  --voxel-snapshot "${VOXEL}" --planning-config "${PLANNING_CONFIG}" \
  --scene-graph "${SCENE_GRAPH}" --task-graph "${TASK_GRAPH}" \
  --planning-dir "${PLANNING_DIR}" --output "${VALIDATION}"

if [[ -s "${BOXER_SCENE}/boxer_3dbbs_fused.csv" ]]; then
  echo "PASS: complete Stage-2 offline package is ready: ${VALIDATION}"
else
  echo "PASS: Stage-2 pipeline smoke is ready (partial, unfused Boxer): ${VALIDATION}"
fi
