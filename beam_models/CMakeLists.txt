cmake_minimum_required(VERSION 3.14)
project(beam_models)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(beam REQUIRED COMPONENTS calibration cv utils matching filtering)
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
    std_srvs
    tf2
    tf2_2d
    tf2_geometry_msgs
    tf2_ros
    beam_constraints
    beam_slam_common
		beam_variables
    cv_bridge
    slamtools
    basalt)

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(Catch2 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS 
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
    sensor_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
    beam_constraints
    beam_slam_common
		beam_variables
    cv_bridge
    slamtools
    basalt
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
  src/motion/unicycle_3d.cpp
  src/motion/unicycle_3d_ignition.cpp
  src/frame_to_frame/frame_to_frame_sensor_model_base.cpp
  src/frame_to_frame/scan_matcher_3d.cpp
  src/frame_to_frame/multi_scan_registration.cpp
	src/frame_to_frame/imu_3d.cpp
	src/frame_to_frame/imu_preintegration.cpp
  src/frame_to_frame/imu_state.cpp
  src/frame_initializers/frame_initializer_base.cpp
  src/frame_initializers/odometry_frame_initializer.cpp
  src/frame_initializers/pose_file_frame_initializer.cpp
  src/camera_to_camera/visual_odom.cpp
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
    beam::calibration
    beam::cv
)
add_dependencies(
  ${PROJECT_NAME}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)


#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  add_executable(${PROJECT_NAME}_scan_matcher_3d_gtests 
    tests/scan_matcher_3d_tests.cpp)
  target_link_libraries(${PROJECT_NAME}_scan_matcher_3d_gtests 
    ${PROJECT_NAME}
    gtest_main
    beam::utils)
  add_test(NAME ${PROJECT_NAME}_scan_matcher_3d_gtests 
          COMMAND ${PROJECT_NAME}_scan_matcher_3d_gtests)


  find_package(rostest REQUIRED)
  find_package(basalt REQUIRED)

  # Lint tests
  roslint_add_test()

  # Model tests
  catkin_add_gtest(imu_preintegration_tests
    tests/imu_preintegration_tests.cpp
  )
  target_link_libraries(imu_preintegration_tests
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )
  set_target_properties(imu_preintegration_tests
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )
endif()

