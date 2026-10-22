#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /workspace/falcon_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11479 ROS_HOSTNAME=127.0.0.1
out=/workspace/shared/outputs/consistency_validation_20261015
roscore -p 11479 > "$out/path_cost_fixed_master.log" 2>&1 &
master_pid=$!
trap 'kill -INT "$master_pid" 2>/dev/null || true; wait "$master_pid" || true' EXIT
for attempt in $(seq 1 40); do if rosparam list >/dev/null 2>&1; then break; fi; sleep .2; done
planner=/workspace/falcon_ws/src/FALCON/falcon_planner
for relative in exploration_manager/config/robot/uav_model_simulator.yaml voxel_mapping/config/voxel_mapping.yaml pathfinding/config/astar.yaml fast_planner/config/fast_planner.yaml exploration_preprocessing/config/frontier_finder.yaml exploration_preprocessing/config/hgrid.yaml exploration_utils/config/perception_utils.yaml exploration_manager/config/exploration_manager.yaml; do rosparam load "$planner/$relative"; done
rosparam load /workspace/shared/runtime/consistency_validation_20261015/A3_observation_lifecycle/config.yaml
rosparam set /fast_planner/observed_trajectory_guard false
rosparam set /fast_planner/consistent_path_clearance false
/tmp/path_cost_fixed_map_profile > "$out/path_cost_fixed_map_profile.log" 2>&1
python3 -c 'from pathlib import Path; import json; p=Path("/workspace/shared/outputs/consistency_validation_20261015"); rows=json.loads(next(l for l in (p/"path_cost_fixed_map_profile.log").read_text().splitlines() if l.startswith("PROFILE_RESULT ")).split(" ",1)[1]); result={"scope":"fixed synthetic map, actual production code; no Habitat or historical replay", "phases":["ray","guard_and_parameters","reset","search","path_extraction"],"rows":rows}; (p/"path_cost_fixed_map_profile.json").write_text(json.dumps(result,indent=2)); print(json.dumps(result))'
