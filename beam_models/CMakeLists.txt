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
    std_msgs
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
    message_generation
    beam_constraints
    beam_slam_common
		beam_variables
    cv_bridge
    basalt)

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(Catch2 REQUIRED)

add_message_files(
  DIRECTORY
    msg
  FILES
    InitializedPathMsg.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

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
    std_msgs
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
  src/frame_to_frame/scan_registration/multi_scan_registration.cpp
  src/frame_to_frame/scan_registration/scan_to_map_registration.cpp
	src/frame_to_frame/imu_preintegration.cpp
  src/frame_to_frame/imu_state.cpp
  src/frame_initializers/frame_initializer_base.cpp
  src/frame_initializers/odometry_frame_initializer.cpp
  src/frame_initializers/pose_file_frame_initializer.cpp
  src/trajectory_initializers/imu_initializer.cpp
  src/trajectory_initializers/vio_initializer.cpp
  src/camera_to_camera/visual_inertial_odom.cpp
  src/camera_to_camera/visual_map.cpp
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
  find_package(rostest REQUIRED)
  find_package(basalt REQUIRED)
  # Lint tests
  roslint_add_test()
  
  # Scan to scan registration tests
  catkin_add_gtest(multi_scan_registration_tests 
    tests/multi_scan_registration_tests.cpp
  )
  target_link_libraries(multi_scan_registration_tests 
    ${PROJECT_NAME}
  )
  set_target_properties(multi_scan_registration_tests 
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )  

  # Scan to map registration tests
  catkin_add_gtest(scan_to_map_registration_tests 
    tests/scan_to_map_registration_tests.cpp
  )
  target_link_libraries(scan_to_map_registration_tests 
    ${PROJECT_NAME}
  )
  set_target_properties(scan_to_map_registration_tests 
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # IMU preintegration tests
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


  add_executable(${PROJECT_NAME}_reprojection_test
    tests/reprojection_test.cpp)
  target_link_libraries(${PROJECT_NAME}_reprojection_test
    ${PROJECT_NAME}
    gtest_main
    beam::utils
    beam::cv)
  add_test(NAME ${PROJECT_NAME}_reprojection_test
          COMMAND ${PROJECT_NAME}_reprojection_test)

endif()

