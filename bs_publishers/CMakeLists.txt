cmake_minimum_required(VERSION 2.8.3)
project(bs_publishers)

set(build_depends
  fuse_core
  fuse_variables
  bs_models
  geometry_msgs
  nav_msgs
  pluginlib
  roscpp
  tf2
  tf2_geometry_msgs
  tf2_ros
  )

find_package(catkin REQUIRED COMPONENTS
  ${build_depends}
  )

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  ${PROJECT_NAME}
  CATKIN_DEPENDS
  ${build_depends}
)

###########
## Build ##
###########

add_compile_options(-std=c++14)

# fuse_publishers library
add_library(${PROJECT_NAME}
  src/path_3d_publisher.cpp
  src/odometry_3d_publisher.cpp
  )
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
  )
target_include_directories(${PROJECT_NAME}
  PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
  )
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )
