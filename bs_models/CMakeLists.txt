cmake_minimum_required(VERSION 3.14)
project(bs_models)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(
  beam REQUIRED 
  COMPONENTS 
  calibration
  cv
  utils
  matching
  filtering)

find_package(
  catkin REQUIRED
  COMPONENTS
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
    )

find_package(gflags REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV 4.5.2 REQUIRED COMPONENTS)
find_package(basalt REQUIRED)

find_package(fuse_core REQUIRED)
find_package(fuse_publishers REQUIRED)
find_package(fuse_variables REQUIRED)
find_package(fuse_graphs REQUIRED)
find_package(fuse_models REQUIRED)
find_package(fuse_constraints REQUIRED)

find_package(bs_constraints REQUIRED)
find_package(bs_common REQUIRED)
find_package(bs_variables REQUIRED)


catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS 
    std_msgs
    geometry_msgs
    message_runtime
    nav_msgs
    pluginlib
    roscpp
    sensor_msgs
    tf2
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
  ${OpenCV_INCLUDE_DIRS}
  ${basalt_INCLUDE_DIRS}
  ${fuse_graphs_INCLUDE_DIRS}
  ${fuse_models_INCLUDE_DIRS}
  ${fuse_publishers_INCLUDE_DIRS}
  ${fuse_variables_INCLUDE_DIRS}
  ${fuse_constraints_INCLUDE_DIRS}
  ${bs_constraints_INCLUDE_DIRS}
  ${bs_common_INCLUDE_DIRS}
  ${bs_variables_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(
  ${PROJECT_NAME}
  src/motion/unicycle_3d.cpp
  src/motion/unicycle_3d_ignition.cpp
  src/frame_to_frame/scan_matcher_3d.cpp
  src/frame_to_frame/scan_registration/scan_registration_base.cpp
  src/frame_to_frame/scan_registration/multi_scan_registration.cpp
  src/frame_to_frame/scan_registration/scan_to_map_registration.cpp
  src/frame_to_frame/imu_preintegration.cpp
  src/frame_initializers/frame_initializer_base.cpp
  src/frame_initializers/odometry_frame_initializer.cpp
  src/frame_initializers/pose_file_frame_initializer.cpp
  src/trajectory_initializers/imu_initializer.cpp
  src/trajectory_initializers/vio_initializer.cpp
  src/trajectory_initializers/lo_initializer.cpp
  src/camera_to_camera/visual_inertial_odom.cpp
  src/camera_to_camera/visual_map.cpp
  src/camera_to_camera/keyframe.cpp
  src/global_mapping/global_mapper.cpp
  src/global_mapping/global_map.cpp
  src/global_mapping/submap.cpp
  src/global_mapping/loop_closure/loop_closure_candidate_search_eucdist.cpp
  src/global_mapping/loop_closure/loop_closure_refinement_loam_registration.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
  ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
    ${basalt_LIBRARIES}
    ${fuse_graphs_LIBRARIES}
    ${fuse_models_LIBRARIES}
    ${fuse_publishers_LIBRARIES}
    ${fuse_variables_LIBRARIES}
    ${fuse_constraints_LIBRARIES}
    ${bs_constraints_LIBRARIES}
    ${bs_common_LIBRARIES}
    ${bs_variables_LIBRARIES}
    beam::utils
    beam::matching
    beam::filtering
    beam::mapping
    beam::calibration
    beam::cv
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

  # Reprojection tests
    catkin_add_gtest(reprojection_test
    tests/reprojection_test.cpp
  )
  target_link_libraries(reprojection_test
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )
  set_target_properties(reprojection_test
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

endif()
