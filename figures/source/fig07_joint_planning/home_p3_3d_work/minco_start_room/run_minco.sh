#!/bin/bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /workspace/project/PRE_MAP_VLN-local-data/code/real_fly/stage2_offline/runtime/minco_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1 ROS_HOME=/job/ros
roscore > /job/roscore.log 2>&1 & master=$!
trap 'kill $master 2>/dev/null || true' EXIT
for i in $(seq 1 50); do rosparam list >/dev/null 2>&1 && break; sleep .1; done
rosparam list >/dev/null
for cycle in initial updated; do
 timeout 180 /workspace/project/PRE_MAP_VLN-local-data/code/real_fly/stage2_offline/runtime/minco_ws/devel/lib/mission_planner/full_smooth_mission \
 _preview_only:=true _preview_start_at_first_route_point:=true _exit_after_preview:=true _start_trigger_type:=3 _save_generated_trajectory:=true \
 _saved_trajectory_path:=/job/${cycle}_minco.txt _route_path:=/job/${cycle}_route.txt _known_map_pcd:=/job/collision.pcd _super_config_path:=/job/planner.yaml \
 _use_super_safe_corridor:=true _super_use_recorded_guide_without_astar:=true _super_static_map_only:=true _local_collision_replan_enabled:=false \
 _regional_ceiling/enabled:=true _regional_ceiling/ceiling_z:=1.89 '_regional_ceiling/region_x_min:=[-10.3]' '_regional_ceiling/region_x_max:=[0.3]' '_regional_ceiling/region_y_min:=[-11.3]' '_regional_ceiling/region_y_max:=[-0.9]' _collision_clearance:=0.14 _auto_land:=false _narrow_corridor/enabled:=false _vertical_guide_floor/enabled:=true _guide_tracking/enabled:=false \
 _max_acceleration:=2.0 _max_jerk:=15.0 _max_snap:=100.0 _max_yaw_rate:=0.6 _max_yaw_lock_variation:=1.0 _max_jerk_discontinuity:=0.0001 \
 _yaw_velocity_threshold:=0.08 _yaw_lookahead_time:=0.40 _yaw_start_blend_duration:=1.5 _yaw_terminal_blend_duration:=2.0 > /job/${cycle}_generation.log 2>&1
 test -s /job/${cycle}_minco.txt
 echo "$cycle generated"
done
