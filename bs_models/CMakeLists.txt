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
    rosbag
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
  ${fuse_core_INCLUDE_DIRS}
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
  src/imu_3d.cpp
  src/lo_initializer.cpp
  src/unicycle_3d.cpp
  src/unicycle_3d_ignition.cpp
  src/lidar_odometry.cpp
  src/visual_inertial_odometry.cpp
  src/global_mapper.cpp
  src/lib/vision/visual_map.cpp
  src/lib/vision/keyframe.cpp
  src/lib/global_mapping/global_map.cpp
  src/lib/global_mapping/submap.cpp
  src/lib/global_mapping/global_map_refinement.cpp
  src/lib/loop_closure/loop_closure_candidate_search_eucdist.cpp
  src/lib/loop_closure/loop_closure_refinement_loam_registration.cpp
  src/lib/scan_registration/scan_registration_base.cpp
  src/lib/scan_registration/multi_scan_registration.cpp
  src/lib/scan_registration/scan_to_map_registration.cpp
  src/lib/scan_registration/registration_map.cpp
  src/lib/frame_initializers/frame_initializer_base.cpp
  src/lib/frame_initializers/odometry_frame_initializer.cpp
  src/lib/frame_initializers/pose_file_frame_initializer.cpp
  src/lib/trajectory_initializers/imu_initializer.cpp
  src/lib/trajectory_initializers/vio_initializer.cpp
  src/lib/scan_pose.cpp
  src/lib/current_submap.cpp
  src/lib/imu_preintegration.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
  ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
    ${fuse_core_LIBRARIES}
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

  # Lint tests
  roslint_add_test()

  # scan pose tests
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
  
  # Scan to scan registration tests
  catkin_add_gtest(${PROJECT_NAME}_multi_scan_registration_tests 
    tests/multi_scan_registration_tests.cpp
  )
  target_link_libraries(${PROJECT_NAME}_multi_scan_registration_tests 
    ${PROJECT_NAME}
  )
  set_target_properties(${PROJECT_NAME}_multi_scan_registration_tests 
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )  

  # Scan to map registration tests
  catkin_add_gtest(${PROJECT_NAME}_scan_to_map_registration_tests 
    tests/scan_to_map_registration_tests.cpp
  )
  target_link_libraries(${PROJECT_NAME}_scan_to_map_registration_tests 
    ${PROJECT_NAME}
  )
  set_target_properties(${PROJECT_NAME}_scan_to_map_registration_tests 
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # IMU preintegration tests
  find_package(basalt REQUIRED)
  catkin_add_gtest(${PROJECT_NAME}_imu_preintegration_tests
    tests/imu_preintegration_tests.cpp
  )
  target_include_directories(${PROJECT_NAME}_imu_preintegration_tests
    PUBLIC
    tests/include
    ${basalt_INCLUDE_DIRS}
  )
  target_link_libraries(${PROJECT_NAME}_imu_preintegration_tests
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
    ${basalt_LIBRARIES}
  )
  set_target_properties(${PROJECT_NAME}_imu_preintegration_tests
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # Reprojection tests
  catkin_add_gtest(${PROJECT_NAME}_reprojection_test
    tests/reprojection_test.cpp
  )
  target_link_libraries(${PROJECT_NAME}_reprojection_test
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )
  set_target_properties(${PROJECT_NAME}_reprojection_test
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # global map refinement tests
  add_executable(${PROJECT_NAME}_global_map_refinement_tests 
    tests/global_map_refinement_tests.cpp)
  target_link_libraries(${PROJECT_NAME}_global_map_refinement_tests 
    ${PROJECT_NAME}
    gtest_main
    beam::utils
    beam::filtering
    beam::matching
  )
  add_test(NAME ${PROJECT_NAME}_global_map_refinement_tests 
          COMMAND ${PROJECT_NAME}_global_map_refinement_tests)

endif()
