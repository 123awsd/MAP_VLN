cmake_minimum_required(VERSION 2.8)
project(traj_server)

# set(CMAKE_VERBOSE_MAKEFILE "false")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native -DEIGEN_DONT_PARALLELIZE ${CMAKE_CXX_FLAGS} -g -O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    quadrotor_msgs
    visualization_msgs
    tf)

find_package(Eigen3 REQUIRED)

catkin_package(
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
#    ${PCL_INCLUDE_DIRS}
    include
    )

# link_directories(
#     ${PCL_LIBRARY_DIRS})

add_executable(eight_trajectory_cmd_node
    src/eight_trajectory_cmd_node.cpp)

add_dependencies(eight_trajectory_cmd_node
    ${catkin_EXPORTED_TARGETS})

target_link_libraries(eight_trajectory_cmd_node
    ${catkin_LIBRARIES}
    )

add_executable(circle_trajectory_cmd_node
    src/circle_trajectory_cmd_node.cpp)

add_dependencies(circle_trajectory_cmd_node
    ${catkin_EXPORTED_TARGETS})

target_link_libraries(circle_trajectory_cmd_node
    ${catkin_LIBRARIES}
    )

catkin_install_python(PROGRAMS
    scripts/analyze_eight_tracking_log.py
    scripts/analyze_circle_tracking_log.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
