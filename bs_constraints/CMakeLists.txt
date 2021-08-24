cmake_minimum_required(VERSION 2.8.3)
project(bs_constraints)

set(build_depends
  fuse_core
  fuse_graphs
  fuse_variables
  fuse_constraints
  bs_variables
  geometry_msgs
  pluginlib
  roscpp
  tf2
  tf2_geometry_msgs
  tf2_ros
  bs_common
  bs_variables
)

find_package(beam REQUIRED COMPONENTS calibration optimization)

find_package(catkin REQUIRED COMPONENTS
  ${build_depends}
)

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${build_depends}
  DEPENDS
    CERES
    EIGEN3
    tf2
    tf2_geometry_msgs
    tf2_ros
)

###########
## Build ##
###########

# fuse_constraints library
add_library(${PROJECT_NAME}
  src/frame_to_frame/relative_constraint.cpp
  src/frame_to_frame/frame_to_frame_transaction_base.cpp
  src/frame_to_frame/imu_state_3d_stamped_transaction.cpp
  src/frame_to_frame/pose_3d_stamped_transaction.cpp
  src/frame_to_frame/relative_imu_state_3d_stamped_constraint.cpp
  src/global/absolute_constraint.cpp
  src/global/absolute_imu_state_3d_stamped_constraint.cpp
  src/motion/unicycle_3d_state_kinematic_constraint.cpp
  src/camera_to_camera/visual_constraint.cpp
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  beam::calibration
  beam::matching
  beam::optimization
)

##########
## Test ##
##########

if(CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  find_package(rostest REQUIRED)

  # Lint tests
  set(ROSLINT_CPP_OPTS "--filter=-build/c++11,-runtime/references")
  roslint_cpp()
  roslint_add_test()

  # Absolute Imu State Stamped Constraint Tests
  catkin_add_gtest(test_absolute_imu_state_3d_stamped_constraint
    tests/test_absolute_imu_state_3d_stamped_constraint.cpp
  )
  add_dependencies(test_absolute_imu_state_3d_stamped_constraint
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_absolute_imu_state_3d_stamped_constraint
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
  )
  target_link_libraries(test_absolute_imu_state_3d_stamped_constraint
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )
  set_target_properties(test_absolute_imu_state_3d_stamped_constraint
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # Relative Imu State Stamped Constraint Tests
  #
  # testing has been moved to imu_preintegration_tests.cpp in bs_models
endif()
