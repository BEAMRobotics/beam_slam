cmake_minimum_required(VERSION 2.8.3)
project(bs_common)

find_package(tf REQUIRED)
find_package(beam REQUIRED COMPONENTS utils filtering matching calibration)

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
    bs_variables
    message_generation
  )

add_message_files(
  DIRECTORY
    msg
  FILES
    InitializedPathMsg.msg
    LandmarkMeasurementMsg.msg
    CameraMeasurementMsg.msg    
    LidarMeasurementMsg.msg
    TrajectoryMeasurementMsg.msg
    SlamChunkMsg.msg
    SubmapMsg.msg
    RelocRequestMsg.msg
    DescriptorMsg.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
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
    bs_variables
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
  src/bs_common/extrinsics_lookup_base.cpp
  src/bs_common/extrinsics_lookup_online.cpp
  src/bs_common/imu_state.cpp
  src/bs_common/pose_lookup.cpp
  src/bs_common/preintegrator.cpp
  src/bs_common/utils.cpp
  src/bs_common/bs_msgs.cpp
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
    beam::matching
  )

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  # Imu State tests
  catkin_add_gtest(${PROJECT_NAME}_imu_state_tests_tests 
    tests/imu_state_tests.cpp
  )
  target_link_libraries(${PROJECT_NAME}_imu_state_tests_tests 
    ${PROJECT_NAME}
  )
  set_target_properties(${PROJECT_NAME}_imu_state_tests_tests 
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )  

endif()