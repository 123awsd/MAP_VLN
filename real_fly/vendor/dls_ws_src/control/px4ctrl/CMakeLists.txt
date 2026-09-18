cmake_minimum_required(VERSION 2.8.3)
project(px4ctrl)

set(CMAKE_VERBOSE_MAKEFILE "false")

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  quadrotor_msgs
  geometry_msgs
  sensor_msgs
  uav_utils
  mavros
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(Eigen3 REQUIRED) 

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    quadrotor_msgs
    geometry_msgs
    sensor_msgs
    uav_utils
    mavros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  include/${PROJECT_NAME}
  include
)

add_executable(px4ctrl_node 
  src/px4ctrl_node.cpp
  src/PX4CtrlFSM.cpp
  src/PX4CtrlParam.cpp
  src/controller.cpp
  src/input.cpp
)

# Px4ctrlDebug.h, PositionCommand.h and TakeoffLand.h are generated from the
# quadrotor_msgs package. Keep the explicit dependency here so a clean,
# parallel build cannot compile px4ctrl before those headers are generated.
add_dependencies(px4ctrl_node
  ${catkin_EXPORTED_TARGETS}
  ${quadrotor_msgs_EXPORTED_TARGETS}
)

target_link_libraries(px4ctrl_node
  ${catkin_LIBRARIES}
)

add_executable(px4debugreplay src/debug_replay.cpp)
add_dependencies(px4debugreplay
  ${catkin_EXPORTED_TARGETS}
  ${quadrotor_msgs_EXPORTED_TARGETS}
)
target_link_libraries(px4debugreplay ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS thrust_calibrate_scrips/thrust_calibrate.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
