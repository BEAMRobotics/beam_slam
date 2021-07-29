cmake_minimum_required(VERSION 2.8.3)
project(bs_common)

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
    std_msgs
    message_generation
  )


add_message_files(
  DIRECTORY
    msg
  FILES
    LandmarkMeasurementMsg.msg
    CameraMeasurementMsg.msg    
    LidarMeasurementMsg.msg
    TrajectoryMeasurementMsg.msg
    SlamChunkMsg.msg
    SubmapMsg.msg
    RelocRequestMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
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
    std_msgs
    geometry_msgs
    message_runtime
)

###########
## Build ##
###########

add_compile_options(-std=c++14)

# library
add_library(${PROJECT_NAME}
  src/bs_parameters/params.cpp
  src/bs_parameters/models/model_params.cpp
  src/bs_parameters/publishers/publisher_params.cpp
  src/bs_common/common.cpp
  src/bs_common/extrinsics_lookup.cpp
  src/bs_common/pose_lookup.cpp
  src/bs_common/scan_pose.cpp
  src/bs_common/lidar_map.cpp
  src/bs_common/preintegrator.cpp
  src/bs_common/utils.cpp
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
