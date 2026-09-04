cmake_minimum_required(VERSION 3.0.2)
project(stage1_fast_lio)

set(DLS_FAST_LIO_SOURCE "/home/nv/dls_ws/src/localization/FAST_LIO")
set(DLS_FAST_LIO_DEVEL "/home/nv/dls_ws/devel")
get_filename_component(STAGE1_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(STAGE1_LIO_ROOT "${STAGE1_ROOT}/runtime/fast_lio" CACHE PATH "Per-run FAST-LIO output root")

if(NOT EXISTS "${DLS_FAST_LIO_SOURCE}/src/laserMapping.cpp")
  message(FATAL_ERROR "Read-only FAST-LIO source was not found at ${DLS_FAST_LIO_SOURCE}")
endif()
if(NOT EXISTS "${DLS_FAST_LIO_DEVEL}/include/fast_lio/LioDebug.h")
  message(FATAL_ERROR "Generated FAST-LIO message headers were not found at ${DLS_FAST_LIO_DEVEL}")
endif()

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf
  tf2_ros
  pcl_ros
  pcl_conversions
  livox_ros_driver2
  eigen_conversions
  cv_bridge
)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenMP QUIET)

catkin_package(CATKIN_DEPENDS
  geometry_msgs nav_msgs roscpp rospy std_msgs sensor_msgs tf tf2_ros
  pcl_ros pcl_conversions livox_ros_driver2 eigen_conversions cv_bridge
)

add_definitions(-DROOT_DIR=\"${STAGE1_LIO_ROOT}/\")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread -fexceptions")
if(OpenMP_CXX_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${DLS_FAST_LIO_SOURCE}/include
  ${DLS_FAST_LIO_DEVEL}/include
)
add_definitions(${PCL_DEFINITIONS})

add_executable(fastlio_mapping
  src/laserMapping.cpp
  ${DLS_FAST_LIO_SOURCE}/src/preprocess.cpp
  ${DLS_FAST_LIO_SOURCE}/include/ikd-Tree/ikd_Tree.cpp
)
add_dependencies(fastlio_mapping ${catkin_EXPORTED_TARGETS})
target_link_libraries(fastlio_mapping
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)
if(OpenMP_CXX_FOUND)
  target_link_libraries(fastlio_mapping OpenMP::OpenMP_CXX)
endif()

install(TARGETS fastlio_mapping
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
