cmake_minimum_required(VERSION 2.8.3)
project(beam_slam_common)

set(build_depends
  roscpp
  )

find_package(tf REQUIRED)
find_package(beam REQUIRED COMPONENTS utils filtering matching)

find_package(
  catkin REQUIRED 
  COMPONENTS
    ${build_depends}
    fuse_variables
    fuse_constraints
    fuse_models
    sensor_msgs
    geometry_msgs
  )

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${build_depends}
    fuse_variables
    fuse_constraints
    fuse_models
    sensor_msgs
    geometry_msgs
)

###########
## Build ##
###########

add_compile_options(-std=c++14)

# library
add_library(${PROJECT_NAME}
  src/beam_parameters/params.cpp
  src/beam_parameters/models/model_params.cpp
  src/beam_parameters/publishers/publisher_params.cpp
  src/beam_common/common.cpp
  src/beam_common/extrinsics_lookup.cpp
  src/beam_common/pose_lookup.cpp
  src/beam_common/scan_pose.cpp
  src/beam_common/lidar_map.cpp
  src/beam_common/preintegrator.cpp
  )
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
  )
target_include_directories(${PROJECT_NAME}
  PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
  )
target_link_libraries(
  ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${tf_LIBRARIES}
    beam::utils
  )

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  add_executable(${PROJECT_NAME}_scan_pose_tests 
    tests/scan_pose_tests.cpp)
  target_link_libraries(${PROJECT_NAME}_scan_pose_tests 
    ${PROJECT_NAME}
    gtest_main
    beam::utils
    beam::filtering
    beam::matching)
  add_test(NAME ${PROJECT_NAME}_scan_pose_tests 
          COMMAND ${PROJECT_NAME}_scan_pose_tests)

endif()
