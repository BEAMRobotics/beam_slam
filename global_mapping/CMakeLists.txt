cmake_minimum_required(VERSION 3.14)
project(global_mapping)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(beam REQUIRED COMPONENTS utils matching filtering cv calibration containers mapping)

find_package(
  catkin REQUIRED
  COMPONENTS
    fuse_core
    fuse_graphs
    fuse_publishers
    fuse_variables
    fuse_models
    fuse_constraints
    geometry_msgs
    nav_msgs
    pluginlib
    rosbag
    roscpp
    roslint
    sensor_msgs
    std_msgs
    tf2
    tf2_2d
    tf2_geometry_msgs
    tf2_ros
    message_generation
    beam_constraints
    beam_models
    beam_slam_common)

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(Catch2 REQUIRED)

add_message_files(
  DIRECTORY
    msg
  FILES
    LandmarkMeasurementMsg.msg
    CameraMeasurementMsg.msg    
    LidarMeasurementMsg.msg
    TrajectoryMeasurementMsg.msg
    SlamChunkMsg.msg
)

generate_messages()

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    fuse_core
    fuse_graphs
    fuse_models
    fuse_publishers
    fuse_variables
    fuse_constraints
    geometry_msgs
    std_msgs
    message_runtime
    nav_msgs
    pcl_conversions
    pluginlib
    roscpp
    sensor_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
    beam_constraints
    beam_models
    beam_slam_common
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

## Declare a C++ library
add_library(
  ${PROJECT_NAME}
  src/global_mapper.cpp
  src/global_map.cpp
  src/submap.cpp
  src/loop_closure/loop_closure_candidate_search_eucdist.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
  ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
    beam::utils
    beam::matching
    beam::filtering
    beam::mapping
    beam::cv
    beam::calibration
    beam::containers
    beam::mapping
)
add_dependencies(
  ${PROJECT_NAME}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)


#############
## Testing ##
#############
add_executable(${PROJECT_NAME}_global_mapping_gtests 
  tests/global_mapping_tests.cpp)
target_link_libraries(${PROJECT_NAME}_global_mapping_gtests 
  ${PROJECT_NAME}
  gtest_main
  beam::utils)
add_test(NAME ${PROJECT_NAME}_global_mapping_gtests 
         COMMAND ${PROJECT_NAME}_global_mapping_gtests)