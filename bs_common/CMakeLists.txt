cmake_minimum_required(VERSION 3.14)
project(bs_common)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(
  beam REQUIRED
  COMPONENTS 
  utils 
  filtering 
  matching 
  calibration
)

set(catkin_build_depends
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    tf
    fuse_variables
    fuse_constraints
    bs_variables
)

find_package(
  catkin REQUIRED 
  COMPONENTS
    ${catkin_build_depends}
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
    ${catkin_build_depends}
    message_runtime
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
  ${catkin_INCLUDE_DIRS}
  ${beam_INCLUDE_DIRS}
)

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
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(
  ${PROJECT_NAME}
    ${catkin_LIBRARIES}
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