cmake_minimum_required(VERSION 3.14)
project(bs_models)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_BUILD_TYPE "release")

find_package(Eigen3 3.3.7 REQUIRED)

find_package(
  beam REQUIRED 
  COMPONENTS 
  calibration
  cv
  utils
  matching
  mapping
  filtering
  optimization
)

set(catkin_build_depends
    std_msgs
    std_srvs
    geometry_msgs
    nav_msgs
    pluginlib
    rosbag
    roscpp
    sensor_msgs
    tf2
    fuse_core
    fuse_graphs
    fuse_variables
    fuse_constraints
    bs_common
    bs_constraints
    bs_variables
)

find_package(
  catkin REQUIRED
  COMPONENTS
    ${catkin_build_depends}
    message_generation
)

find_package(OpenCV 4.5.2 REQUIRED COMPONENTS)

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

## Declare a C++ library
add_library(
  ${PROJECT_NAME}
  ## sensor models
  src/lidar_scan_deskewer.cpp
  src/unicycle_3d.cpp
  src/lidar_odometry.cpp
  src/visual_odometry.cpp
  src/inertial_odometry.cpp
  src/gravity_alignment.cpp
  src/slam_initialization.cpp
  src/visual_feature_tracker.cpp
  src/global_mapper.cpp
  src/graph_visualization.cpp
  src/graph_publisher.cpp
  ## vision helpers
  src/lib/vision/visual_map.cpp
  src/lib/vision/keyframe.cpp
  src/lib/vision/utils.cpp
  src/lib/vision/vo_localization_validation.cpp
  ## imu helpers
  src/lib/imu/imu_preintegration.cpp
  src/lib/imu/inertial_alignment.cpp
  ## lidar helpers
  src/lib/lidar/lidar_path_init.cpp
  src/lib/lidar/scan_pose.cpp
  ## global mapping
  src/lib/global_mapping/global_map.cpp
  src/lib/global_mapping/submap.cpp
  src/lib/global_mapping/global_map_refinement.cpp
  src/lib/global_mapping/submap_refinement.cpp
  src/lib/global_mapping/submap_alignment.cpp
  src/lib/global_mapping/submap_pose_graph_optimization.cpp
  src/lib/global_mapping/global_map_batch_optimization.cpp
  src/lib/global_mapping/utils.cpp
  ## relocalization
  src/lib/reloc/reloc_candidate_search_base.cpp
  src/lib/reloc/reloc_refinement_base.cpp
  src/lib/reloc/reloc_candidate_search_eucdist.cpp
  src/lib/reloc/reloc_candidate_search_scan_context.cpp
  src/lib/reloc/reloc_refinement_loam_registration.cpp
  ## scan registration
  src/lib/scan_registration/scan_registration_base.cpp
  src/lib/scan_registration/multi_scan_registration.cpp
  src/lib/scan_registration/scan_to_map_registration.cpp
  src/lib/scan_registration/registration_map.cpp
  src/lib/scan_registration/registration_validation.cpp
  ## frame initializers
  src/lib/frame_initializers/frame_initializer.cpp
  # graph visualization
  src/lib/graph_visualization/helpers.cpp
  ## experimental
  experimental/src/lidar_aggregation
  experimental/src/lib/lidar/lidar_aggregator
)

include_directories(
  include
  experimental/include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
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

## Specify libraries to link a library or executable target against
target_link_libraries(
  ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    beam::utils
    beam::matching
    beam::filtering
    beam::mapping
    beam::calibration
    beam::cv
    beam::optimization
)

if(NOT CMAKE_DISABLE_EXPERIMENTAL)
    add_subdirectory(experimental)
endif(NOT CMAKE_DISABLE_EXPERIMENTAL)

#############
## Testing ##
#############
# to build tests, run catkin build with: --make-args tests
if(CATKIN_ENABLE_TESTING)
  # scan pose tests
  catkin_add_gtest(${PROJECT_NAME}_scan_pose_tests 
    tests/scan_pose_tests.cpp
  )
  target_link_libraries(${PROJECT_NAME}_scan_pose_tests 
    ${PROJECT_NAME}
  )
  set_target_properties(${PROJECT_NAME}_scan_pose_tests 
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )  
  
  # Scan to scan registration tests
  catkin_add_gtest(${PROJECT_NAME}_multi_scan_registration_tests 
    tests/multi_scan_registration_tests.cpp
  )
  target_link_libraries(${PROJECT_NAME}_multi_scan_registration_tests 
    ${PROJECT_NAME}
  )
  target_include_directories(${PROJECT_NAME}_multi_scan_registration_tests
    PUBLIC
    tests/include
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
  target_include_directories(${PROJECT_NAME}_scan_to_map_registration_tests
    PUBLIC
    tests/include
  )
  set_target_properties(${PROJECT_NAME}_scan_to_map_registration_tests 
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # IMU preintegration tests
  # to build these tests, first install Sophus using function from beam_install_scripts
  # then clone https://github.com/BEAMRobotics/basalt-headers-mirror to your catkin ws
  find_package(basalt QUIET)
  if(basalt_FOUND)
    find_package(fmt)
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
      ${basalt_LIBRARIES}
      fmt::fmt
    )
    set_target_properties(${PROJECT_NAME}_imu_preintegration_tests
      PROPERTIES
        CXX_STANDARD 14
        CXX_STANDARD_REQUIRED YES
    )
  else()
    message( "Basalt not found, not building imu tests." )  
  endif()

  # Reprojection tests
  catkin_add_gtest(${PROJECT_NAME}_reprojection_test
    tests/reprojection_test.cpp
  )
  target_link_libraries(${PROJECT_NAME}_reprojection_test
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )
  set_target_properties(${PROJECT_NAME}_reprojection_test
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # global map refinement tests
  catkin_add_gtest(${PROJECT_NAME}_global_map_refinement_tests
    tests/global_map_refinement_tests.cpp
  )
  target_link_libraries(${PROJECT_NAME}_global_map_refinement_tests
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )
  target_include_directories(${PROJECT_NAME}_global_map_refinement_tests
    PUBLIC
    tests/include
  )
  set_target_properties(${PROJECT_NAME}_global_map_refinement_tests
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

endif()
