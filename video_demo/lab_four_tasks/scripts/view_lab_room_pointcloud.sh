#!/usr/bin/env bash
set -euo pipefail

# One-command RViz preview for the laboratory four-task real-flight map.
# The publisher reads the archived PCD, candidate viewpoints, mission plan,
# semantic targets, and saved trajectory. No flight or sensor node is started.

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
root_dir="$(cd -- "$script_dir/.." && pwd)"

min_z="${RVIZ_MIN_Z:-0.0}"
max_z="${RVIZ_MAX_Z:-2.0}"
frustum_scale="${RVIZ_FRUSTUM_SCALE:-0.25}"
min_observations="${RVIZ_MIN_OBSERVATIONS:-1}"

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<'EOF'
用法：
  scripts/view_lab_room_pointcloud.sh
  scripts/view_lab_room_pointcloud.sh MIN_Z MAX_Z
  scripts/view_lab_room_pointcloud.sh --min-z MIN_Z --max-z MAX_Z

示例：
  scripts/view_lab_room_pointcloud.sh 0.2 2.5
  RVIZ_FRUSTUM_SCALE=0.35 scripts/view_lab_room_pointcloud.sh

默认：点云高度 0.4–2.0 m，显示真实任务轨迹、目标物体和候选观察视锥。
EOF
  exit 0
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --min-z)
      [[ $# -ge 2 ]] || { echo "--min-z 需要一个数值。" >&2; exit 2; }
      min_z="$2"; shift 2 ;;
    --max-z)
      [[ $# -ge 2 ]] || { echo "--max-z 需要一个数值。" >&2; exit 2; }
      max_z="$2"; shift 2 ;;
    --frustum-scale)
      [[ $# -ge 2 ]] || { echo "--frustum-scale 需要一个数值。" >&2; exit 2; }
      frustum_scale="$2"; shift 2 ;;
    --)
      shift; break ;;
    -*)
      echo "未知参数：$1；运行 --help 查看用法。" >&2; exit 2 ;;
    *)
      if [[ -z "${positional_min_z:-}" ]]; then positional_min_z="$1"
      elif [[ -z "${positional_max_z:-}" ]]; then positional_max_z="$1"
      else echo "参数过多；运行 --help 查看用法。" >&2; exit 2; fi
      shift ;;
  esac
done
[[ -n "${positional_min_z:-}" ]] && min_z="$positional_min_z"
[[ -n "${positional_max_z:-}" ]] && max_z="$positional_max_z"

if [[ -f /opt/ros/noetic/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
fi

data_dir="$root_dir/真机实验素材/成功场次/实验室4任务加主动搜索"
map_dir="$data_dir/02_地图与场景"
mission_dir="$data_dir/03_任务规划/multi_object_recovery_flash_07_lounge_z070/planning"
rviz_script="$root_dir/PRE_MAP_VLN/real_fly/stage2_offline/rviz/publish_drone_room_visualization.py"

for required in \
  "$map_dir/fastlio_complete/handheld_map_lab_room_corrected_v1_complete.pcd" \
  "$map_dir/localization/target_points_clustered.json" \
  "$map_dir/localization/target_points_raw.json" \
  "$map_dir/boxer_complete/episode/boxer_3dbbs.csv" \
  "$map_dir/fastlio_complete/trajectory.csv" \
  "$mission_dir/mission_plan.json" \
  "$map_dir/voxel_snapshot"; do
  [[ -e "$required" ]] || { echo "缺少素材：$required" >&2; exit 1; }
done

export RVIZ_MIN_Z="$min_z"
export RVIZ_MAX_Z="$max_z"
export RVIZ_FRUSTUM_SCALE="$frustum_scale"

# The project normally runs ROS/RViz inside its existing Noetic container.
# Do not modify the host environment or pretend that a host ROS install exists.
if ! command -v roscore >/dev/null 2>&1; then
  command -v docker >/dev/null 2>&1 || {
    echo "找不到 roscore，也找不到 docker。请使用项目的 ROS Noetic 容器环境。" >&2
    exit 1
  }
  "$root_dir/PRE_MAP_VLN/scripts/prepare_rviz_xauth.sh" >/dev/null
  exec docker run --rm --init -i --net host \
    --device nvidia.com/gpu=all -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
    -e QT_X11_NO_MITSHM=1 \
    -e DISPLAY="${DISPLAY:-:0}" -e XAUTHORITY=/root/.Xauthority \
    -e RVIZ_TASK_ONLY=1 \
    -e RVIZ_SHOW_BOXES=0 \
    -e RVIZ_SCENE_GRAPH="/workspace/project/真机实验素材/成功场次/实验室4任务加主动搜索/02_地图与场景/approved_scene_graph.json" \
    -e RVIZ_MIN_OBSERVATIONS="$min_observations" \
    -e RVIZ_CANDIDATES_JSON="/workspace/project/视频制作/实验室4任务_候选规划预览/candidates.json" \
    -e RVIZ_MIN_Z="$min_z" -e RVIZ_MAX_Z="$max_z" \
    -e RVIZ_FRUSTUM_SCALE="$frustum_scale" \
    -e RVIZ_ANIMATION_SPEED="${RVIZ_ANIMATION_SPEED:-1.0}" \
    -v "$root_dir:/workspace/project:ro" \
    -v "$root_dir/patches:/workspace/project/patches:rw" \
    -v "$root_dir/PRE_MAP_VLN/real_fly/stage2_offline/rviz:/workspace/project/PRE_MAP_VLN/real_fly/stage2_offline/rviz:rw" \
    -v "$root_dir/PRE_MAP_VLN/runtime/rviz.Xauthority:/root/.Xauthority:ro" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -v /tmp:/host_tmp:rw \
    pre-map-vln/falcon-noetic:local bash -lc '
      set -e
      source /opt/ros/noetic/setup.bash
      roscore >/tmp/map_vln_pointcloud_roscore.log 2>&1 & master=$!
      publisher=""; animation=""
      trap "kill $master $publisher $animation 2>/dev/null || true" EXIT INT TERM
      sleep 2
      python3 /workspace/project/PRE_MAP_VLN/real_fly/stage2_offline/rviz/publish_drone_room_visualization.py \
        "/workspace/project/真机实验素材/成功场次/实验室4任务加主动搜索/02_地图与场景/fastlio_complete/handheld_map_lab_room_corrected_v1_complete.pcd" \
        "/workspace/project/真机实验素材/成功场次/实验室4任务加主动搜索/02_地图与场景/localization/target_points_clustered.json" \
        "/workspace/project/真机实验素材/成功场次/实验室4任务加主动搜索/02_地图与场景/localization/target_points_raw.json" \
        "/workspace/project/真机实验素材/成功场次/实验室4任务加主动搜索/02_地图与场景/boxer_complete/episode/boxer_3dbbs.csv" \
        "/workspace/project/真机实验素材/成功场次/实验室4任务加主动搜索/02_地图与场景/fastlio_complete/trajectory.csv" \
        "/workspace/project/真机实验素材/成功场次/实验室4任务加主动搜索/04_最终轨迹/final_minco_preview.json" \
        "$RVIZ_MIN_OBSERVATIONS" \
        "/workspace/project/真机实验素材/成功场次/实验室4任务加主动搜索/02_地图与场景/voxel_snapshot" \
        >/tmp/map_vln_pointcloud_publisher.log 2>&1 & publisher=$!
      echo "等待点云首帧..."
      for _ in $(seq 1 60); do
        if rostopic echo -n1 /drone_room/map >/dev/null 2>&1; then break; fi
        sleep 1
      done
      rostopic echo -n1 /drone_room/map >/dev/null 2>&1 || { cat /tmp/map_vln_pointcloud_publisher.log >&2; exit 1; }
      kill -0 "$publisher" || { cat /tmp/map_vln_pointcloud_publisher.log >&2; exit 1; }
      rviz -d /workspace/project/PRE_MAP_VLN/real_fly/stage2_offline/rviz/lab_task_pointcloud.rviz & rviz_pid=$!
      kill -0 "$rviz_pid" || { echo "RViz 启动失败" >&2; exit 1; }
      gate=/host_tmp/map_vln_start_animation
      rm -f "$gate"
      echo "点云加载完成后，请在另一个终端执行："
      echo "touch $gate"
      while [[ ! -e "$gate" ]]; do sleep 1; done
      rm -f "$gate"
      RVIZ_INITIAL_HOLD_SECONDS=0 python3 /workspace/project/PRE_MAP_VLN/real_fly/stage2_offline/rviz/animate_final_trajectory.py \
        "/workspace/project/真机实验素材/成功场次/实验室4任务加主动搜索/04_最终轨迹/final_minco_preview.json" \
        "/workspace/project/视频制作/实验室4任务_候选规划预览/candidates.json" \
        "/workspace/project/真机实验素材/成功场次/实验室4任务加主动搜索/04_最终轨迹/final_minco.txt" \
        >/tmp/map_vln_pointcloud_animation.log 2>&1 & animation=$!
      wait "$rviz_pid"
    '
fi

if ! command -v rviz >/dev/null 2>&1; then
  echo "找不到 rviz。请先加载项目的 ROS Noetic 环境。" >&2
  exit 1
fi

roscore >/tmp/map_vln_pointcloud_roscore.log 2>&1 &
roscore_pid=$!
publisher_pid=""
cleanup() {
  set +e
  [[ -n "$publisher_pid" ]] && kill "$publisher_pid" 2>/dev/null
  kill "$roscore_pid" 2>/dev/null
  wait "$publisher_pid" 2>/dev/null
  wait "$roscore_pid" 2>/dev/null
}
trap cleanup EXIT INT TERM

sleep 2

python3 "$rviz_script" \
  "$map_dir/fastlio_complete/handheld_map_lab_room_corrected_v1_complete.pcd" \
  "$map_dir/localization/target_points_clustered.json" \
  "$map_dir/localization/target_points_raw.json" \
  "$map_dir/boxer_complete/episode/boxer_3dbbs.csv" \
  "$map_dir/fastlio_complete/trajectory.csv" \
  "$mission_dir/mission_plan.json" \
  "$min_observations" \
  "$map_dir/voxel_snapshot" >/tmp/map_vln_pointcloud_publisher.log 2>&1 &
publisher_pid=$!

echo "等待点云首帧..."
for _ in $(seq 1 60); do
  if rostopic echo -n1 /drone_room/map >/dev/null 2>&1; then break; fi
  sleep 1
done
rostopic echo -n1 /drone_room/map >/dev/null 2>&1 || { cat /tmp/map_vln_pointcloud_publisher.log >&2; exit 1; }

sleep 1
echo "RViz 预览已启动：z=${min_z}–${max_z} m，按 Ctrl-C 退出。"
rviz
