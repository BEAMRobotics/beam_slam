cmake_minimum_required(VERSION 2.8.3)
project(beam_constraints)

set(build_depends
  fuse_core
  fuse_graphs
  fuse_variables
  fuse_constraints
  geometry_msgs
  pluginlib
  roscpp
  tf2
  tf2_geometry_msgs
  tf2_ros
  beam_slam_common
)

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
  src/frame_to_frame/frame_to_frame_transaction_base.cpp
  src/frame_to_frame/pose_3d_stamped_transaction.cpp
  src/motion/unicycle_3d_state_kinematic_constraint.cpp
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
)