cmake_minimum_required(VERSION 3.14)
project(beam_models)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(beam REQUIRED COMPONENTS utils matching)

find_package(
  catkin REQUIRED
  COMPONENTS
    beam_constraints
    fuse_core
    fuse_graphs
    fuse_publishers
    fuse_variables
    fuse_models
    fuse_constraints
    geometry_msgs
    message_generation
    nav_msgs
    pluginlib
    rosbag
    roscpp
    roslint
    sensor_msgs
    std_srvs
    tf2
    tf2_2d
    tf2_geometry_msgs
    tf2_ros)

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    beam_constraints
    fuse_core
    fuse_graphs
    fuse_models
    fuse_publishers
    fuse_variables
    fuse_constraints
    geometry_msgs
    message_runtime
    nav_msgs
    pcl_conversions
    pluginlib
    roscpp
    rosbag
    sensor_msgs
    std_srvs
    tf2
    tf2_2d
    tf2_geometry_msgs
    tf2_ros
  DEPENDS
    Boost
    CERES
    EIGEN3
)

###########
## Build ##
###########
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
message(STATUS "Using libpointmatcher version ${libpointmatcher_VERSION}")
## Declare a C++ library
add_library(
  ${PROJECT_NAME}
  src/motion/unicycle_3d.cpp
  src/motion/unicycle_3d_ignition.cpp
  src/motion/unicycle_3d_state_kinematic_constraint.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
  ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)
