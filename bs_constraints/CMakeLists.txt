cmake_minimum_required(VERSION 2.8.3)
project(bs_constraints)

set(build_depends
  fuse_core
  fuse_graphs
  fuse_variables
  fuse_constraints
  fuse_loss
  bs_variables
  geometry_msgs
  pluginlib
  roscpp
  tf2
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
)

###########
## Build ##
###########

# fuse_constraints library
add_library(${PROJECT_NAME}
  src/relative_pose/relative_constraints.cpp
  src/relative_pose/relative_pose_transaction_base.cpp
  src/relative_pose/imu_state_3d_stamped_transaction.cpp
  src/relative_pose/pose_3d_stamped_transaction.cpp
  src/relative_pose/relative_imu_state_3d_stamped_constraint.cpp
  src/global/absolute_constraint.cpp
  src/global/absolute_imu_state_3d_stamped_constraint.cpp
  src/motion/unicycle_3d_state_kinematic_constraint.cpp
  src/visual/euclidean_reprojection_constraint.cpp
  src/jacobians.cpp
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
  find_package(rostest REQUIRED)

  # Jacobian helper tests
  catkin_add_gtest(${PROJECT_NAME}_jacobian_helper_tests
    tests/jacobian_helper_tests.cpp
  )
  target_link_libraries(${PROJECT_NAME}_jacobian_helper_tests
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )
  set_target_properties(${PROJECT_NAME}_jacobian_helper_tests
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )


  # Absolute Imu State Stamped Constraint Tests
  catkin_add_gtest(${PROJECT_NAME}_absolute_imu_state_3d_stamped_constraint_test
    tests/absolute_imu_state_3d_stamped_constraint_test.cpp
  )
  target_link_libraries(${PROJECT_NAME}_absolute_imu_state_3d_stamped_constraint_test
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )
  set_target_properties(${PROJECT_NAME}_absolute_imu_state_3d_stamped_constraint_test
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

endif()
