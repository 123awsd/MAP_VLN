FROM osrf/ros:noetic-desktop-full

SHELL ["/bin/bash", "-lc"]
ENV DEBIAN_FRONTEND=noninteractive

RUN sed -i \
      -e 's|http://archive.ubuntu.com/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ubuntu|g' \
      -e 's|http://security.ubuntu.com/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ubuntu|g' \
      /etc/apt/sources.list \
    && apt-get -o Acquire::Retries=5 -o Acquire::https::Timeout=30 update \
    && apt-get -o Acquire::Retries=5 -o Acquire::https::Timeout=30 install -y --no-install-recommends \
    build-essential \
    git \
    python3-pip \
    libgoogle-glog-dev \
    libdw-dev \
    libdwarf-dev \
    libarmadillo-dev \
    libc++-dev \
    libc++abi-dev \
    libeigen3-dev \
    libpcl-dev \
    libopencv-dev \
    libboost-all-dev \
    ros-noetic-cmake-modules \
    ros-noetic-cv-bridge \
    ros-noetic-dynamic-reconfigure \
    ros-noetic-image-transport \
    ros-noetic-laser-geometry \
    ros-noetic-nodelet \
    ros-noetic-pcl-conversions \
    ros-noetic-pcl-ros \
    ros-noetic-tf \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir \
    --index-url https://pypi.tuna.tsinghua.edu.cn/simple \
    cmake==3.26.4

COPY third_party/nlopt /tmp/nlopt
RUN cmake -S /tmp/nlopt -B /tmp/nlopt/build \
    -DCMAKE_BUILD_TYPE=Release \
    -DNLOPT_PYTHON=OFF \
    -DNLOPT_OCTAVE=OFF \
    -DNLOPT_MATLAB=OFF \
    -DNLOPT_GUILE=OFF \
    -DNLOPT_SWIG=OFF \
    && cmake --build /tmp/nlopt/build --parallel 4 \
    && cmake --install /tmp/nlopt/build \
    && ldconfig \
    && rm -rf /tmp/nlopt

WORKDIR /workspace/falcon_ws/src
COPY third_party/FALCON /workspace/falcon_ws/src/FALCON
COPY ros_ws/src/pre_map_bridge /workspace/falcon_ws/src/pre_map_bridge
COPY patches/falcon/habitat_ground_viewpoint.patch /tmp/habitat_ground_viewpoint.patch
COPY patches/falcon/true_3d_viewpoints.patch /tmp/true_3d_viewpoints.patch
COPY patches/falcon/astar_start_voxel_escape.patch /tmp/astar_start_voxel_escape.patch
COPY patches/falcon/voxel_astar_fallback.patch /tmp/voxel_astar_fallback.patch
COPY patches/falcon/conditional_safe_astar_segment.patch /tmp/conditional_safe_astar_segment.patch
COPY patches/falcon/zero_cost_guard.patch /tmp/zero_cost_guard.patch
COPY patches/falcon/robust_ground_planner.patch /tmp/robust_ground_planner.patch
COPY patches/falcon/robust_bspline_optimizer.patch /tmp/robust_bspline_optimizer.patch
COPY patches/falcon/exploration_recovery_fallback.patch /tmp/exploration_recovery_fallback.patch
COPY patches/falcon/dormant_frontier_recovery.patch /tmp/dormant_frontier_recovery.patch
COPY patches/falcon/visualization_bspline_guard.patch /tmp/visualization_bspline_guard.patch
COPY patches/falcon/disable_async_bspline_visualization.patch /tmp/disable_async_bspline_visualization.patch
COPY patches/falcon/active_frontier_block.patch /tmp/active_frontier_block.patch
COPY patches/falcon/invalid_frontier_candidate_guard.patch /tmp/invalid_frontier_candidate_guard.patch
COPY patches/falcon/executed_goal_oscillation_guard.patch /tmp/executed_goal_oscillation_guard.patch
COPY patches/falcon/online_floor_priority.patch /tmp/online_floor_priority.patch
COPY patches/falcon/prepublish_collision_viewpoint_block.patch /tmp/prepublish_collision_viewpoint_block.patch
COPY patches/falcon/unreachable_hgrid_relocation.patch /tmp/unreachable_hgrid_relocation.patch
COPY patches/falcon/sop_timeout_guard.patch /tmp/sop_timeout_guard.patch
COPY patches/falcon/lidar_radial_depth.patch /tmp/lidar_radial_depth.patch
COPY patches/falcon/lidar_ray_occupancy.patch /tmp/lidar_ray_occupancy.patch
COPY patches/falcon/lidar_unique_gain.patch /tmp/lidar_unique_gain.patch
COPY patches/falcon/coarse_unknown_targets.patch /tmp/coarse_unknown_targets.patch
COPY patches/falcon/refresh_frontier_viewpoints.patch /tmp/refresh_frontier_viewpoints.patch
COPY patches/falcon/scanned_observation_targets.patch /tmp/scanned_observation_targets.patch
COPY patches/falcon/frontier_extrema_candidates.patch /tmp/frontier_extrema_candidates.patch
COPY patches/falcon/asymmetric_lidar_fov.patch /tmp/asymmetric_lidar_fov.patch
COPY patches/falcon/attempted_viewpoint_cooldown.patch /tmp/attempted_viewpoint_cooldown.patch
COPY patches/falcon/progress_aware_viewpoint_cooldown.patch /tmp/progress_aware_viewpoint_cooldown.patch
COPY patches/falcon/preview_trajectory_clearance_guard.patch /tmp/preview_trajectory_clearance_guard.patch
COPY patches/falcon/omni_lidar_visualization_cleanup.patch /tmp/omni_lidar_visualization_cleanup.patch
COPY patches/falcon/recovery_target_memory.patch /tmp/recovery_target_memory.patch
COPY patches/falcon/preview_tracking_recovery.patch /tmp/preview_tracking_recovery.patch
COPY patches/falcon/preview_target_progress.patch /tmp/preview_target_progress.patch
COPY patches/falcon/preview_recovery_height.patch /tmp/preview_recovery_height.patch
RUN patch -d FALCON -p1 < /tmp/habitat_ground_viewpoint.patch
RUN patch -d FALCON -p1 < /tmp/true_3d_viewpoints.patch
RUN patch -d FALCON -p1 < /tmp/astar_start_voxel_escape.patch
RUN patch -d FALCON -p1 < /tmp/voxel_astar_fallback.patch
RUN patch -d FALCON -p1 < /tmp/conditional_safe_astar_segment.patch
RUN patch -d FALCON -p1 < /tmp/zero_cost_guard.patch
RUN patch -d FALCON -p1 < /tmp/robust_ground_planner.patch
RUN patch -d FALCON -p1 < /tmp/robust_bspline_optimizer.patch
RUN if grep -q "No reachable recovery candidate for selected frontier" \
      FALCON/falcon_planner/exploration_manager/src/exploration_manager.cpp; then \
      echo "exploration recovery fallback already present"; \
    else \
      patch -d FALCON -p1 < /tmp/exploration_recovery_fallback.patch; \
    fi
RUN patch -d FALCON -p1 < /tmp/dormant_frontier_recovery.patch
RUN patch -d FALCON -p1 < /tmp/visualization_bspline_guard.patch
RUN patch -d FALCON -p1 < /tmp/disable_async_bspline_visualization.patch
RUN patch -d FALCON -p1 < /tmp/active_frontier_block.patch
RUN patch -d FALCON -p1 < /tmp/invalid_frontier_candidate_guard.patch
RUN patch -d FALCON -p1 < /tmp/executed_goal_oscillation_guard.patch
RUN patch -d FALCON -p1 < /tmp/online_floor_priority.patch
RUN patch -d FALCON -p1 < /tmp/prepublish_collision_viewpoint_block.patch
RUN patch -d FALCON -p1 < /tmp/unreachable_hgrid_relocation.patch
RUN patch -d FALCON -p1 < /tmp/sop_timeout_guard.patch
RUN patch -d FALCON -p1 < /tmp/lidar_radial_depth.patch
RUN patch -d FALCON -p1 < /tmp/lidar_ray_occupancy.patch
RUN patch -d FALCON -p1 < /tmp/lidar_unique_gain.patch
RUN patch -d FALCON -p1 < /tmp/coarse_unknown_targets.patch
RUN patch -d FALCON -p1 < /tmp/refresh_frontier_viewpoints.patch
RUN patch -d FALCON -p1 < /tmp/scanned_observation_targets.patch
RUN patch -d FALCON -p1 < /tmp/frontier_extrema_candidates.patch
RUN patch -d FALCON -p1 < /tmp/asymmetric_lidar_fov.patch
RUN patch -d FALCON -p1 < /tmp/attempted_viewpoint_cooldown.patch
RUN patch -d FALCON -p1 < /tmp/progress_aware_viewpoint_cooldown.patch
RUN patch -d FALCON -p1 < /tmp/preview_trajectory_clearance_guard.patch
RUN patch -d FALCON -p1 < /tmp/omni_lidar_visualization_cleanup.patch
RUN patch -d FALCON -p1 < /tmp/recovery_target_memory.patch
RUN patch -d FALCON -p1 < /tmp/preview_tracking_recovery.patch
RUN patch -d FALCON -p1 < /tmp/preview_target_progress.patch
RUN patch -d FALCON -p1 < /tmp/preview_recovery_height.patch
COPY patches/falcon/preview_start_diagnostics.patch /tmp/preview_start_diagnostics.patch
RUN patch -d FALCON -p1 < /tmp/preview_start_diagnostics.patch
COPY patches/falcon/preview_final_geometry_guard.patch /tmp/preview_final_geometry_guard.patch
RUN patch -d FALCON -p1 < /tmp/preview_final_geometry_guard.patch
COPY patches/falcon/preview_goal_prefilter.patch /tmp/preview_goal_prefilter.patch
RUN patch -d FALCON -p1 < /tmp/preview_goal_prefilter.patch
COPY patches/falcon/preview_geofence.h FALCON/falcon_planner/pathfinding/include/pathfinding/preview_geofence.h
COPY patches/falcon/preview_geofence.patch /tmp/preview_geofence.patch
RUN patch -d FALCON -p1 < /tmp/preview_geofence.patch
COPY patches/falcon/preview_roi_gain.patch /tmp/preview_roi_gain.patch
RUN patch -d FALCON -p1 < /tmp/preview_roi_gain.patch
COPY patches/falcon/preview_scope_consistency.patch /tmp/preview_scope_consistency.patch
RUN patch -d FALCON -p1 < /tmp/preview_scope_consistency.patch
COPY patches/falcon/preview_start_cell_consistency.patch /tmp/preview_start_cell_consistency.patch
RUN patch -d FALCON -p1 < /tmp/preview_start_cell_consistency.patch
COPY patches/falcon/preview_gain_ranking.patch /tmp/preview_gain_ranking.patch
RUN patch -d FALCON -p1 < /tmp/preview_gain_ranking.patch

# Habitat replaces FALCON's built-in CUDA/Open3D sensor renderers.
RUN touch FALCON/uav_simulator/camera_sensing/pointcloud_render/CATKIN_IGNORE \
    FALCON/uav_simulator/camera_sensing/mesh_render/CATKIN_IGNORE \
    FALCON/uav_simulator/map_render/CATKIN_IGNORE

WORKDIR /workspace/falcon_ws
RUN source /opt/ros/noetic/setup.bash \
    && catkin_make -DCMAKE_BUILD_TYPE=Release -j4

COPY containers/falcon_noetic/nvidia-smi-compat /usr/local/bin/nvidia-smi
COPY containers/falcon_noetic/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh /usr/local/bin/nvidia-smi
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
