cmake_minimum_required(VERSION 2.8.3)
project(mission_planner)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -fPIC")
add_compile_options(-DQT_NO_VERSION_TAGGING)
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
add_definitions(-DUSE_ROS1)
add_definitions(-DFMT_HEADER_ONLY)
add_definitions(-DORIGIN_AT_CORNER)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        pcl_ros
        pcl_conversions
        geometry_msgs
        nav_msgs
        sensor_msgs
        message_generation
        mavros_msgs
        quadrotor_msgs
        visualization_msgs
        interactive_markers
        pluginlib
        rviz
        message_filters
        rog_map
        )
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES
        CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs sensor_msgs
        visualization_msgs interactive_markers pluginlib rviz pcl_ros pcl_conversions quadrotor_msgs mavros_msgs
        rog_map
        #  DEPENDS system_lib
)

include_directories(
        SYSTEM
        include
        ../super_planner/include
        ../rog_map/include
        ../../../devel/include
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${Qt5Widgets_INCLUDE_DIRS}
)

set(THIRD_PARTY
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        yaml-cpp
        -ldw
        )

add_executable(waypoint_mission
        Apps/ros1_waypoint_mission.cpp
        )
add_dependencies(waypoint_mission ${catkin_EXPORTED_TARGETS})
target_link_libraries(waypoint_mission
        ${THIRD_PARTY}
        )

add_executable(waypoint_recorder
        Apps/ros1_waypoint_recorder.cpp
        )
add_dependencies(waypoint_recorder ${catkin_EXPORTED_TARGETS})
target_link_libraries(waypoint_recorder
        ${THIRD_PARTY}
        )

add_library(waypoint_recorder_panel
        Apps/waypoint_recorder_panel.cpp
        )
add_dependencies(waypoint_recorder_panel ${catkin_EXPORTED_TARGETS})
target_link_libraries(waypoint_recorder_panel
        ${THIRD_PARTY}
        Qt5::Widgets
        )

add_executable(competition_cloud_guard
        Apps/ros1_competition_cloud_guard.cpp
        )
add_dependencies(competition_cloud_guard ${catkin_EXPORTED_TARGETS})
target_link_libraries(competition_cloud_guard
        ${THIRD_PARTY}
        )

add_executable(competition_command_mux
        Apps/ros1_competition_command_mux.cpp
        )
add_dependencies(competition_command_mux ${catkin_EXPORTED_TARGETS})
target_link_libraries(competition_command_mux
        ${THIRD_PARTY}
        )

add_executable(full_smooth_mission
        Apps/ros1_full_smooth_mission.cpp
        ../super_planner/src/utils/banded_system.cpp
        ../super_planner/src/utils/minco.cpp
        ../super_planner/src/utils/piece.cpp
        ../super_planner/src/utils/root_finder.cpp
        ../super_planner/src/utils/trajectory.cpp
        )
add_dependencies(full_smooth_mission ${catkin_EXPORTED_TARGETS})
target_link_libraries(full_smooth_mission
        # super_planner is declared in package.xml.  Its ROS1 export library is
        # named `super`, so it is linked directly instead of as a catkin
        # COMPONENT (which would incorrectly search for libsuper_planner).
        super
        rog_map
        ${THIRD_PARTY}
        )
set_property(TARGET full_smooth_mission PROPERTY CXX_STANDARD 17)
