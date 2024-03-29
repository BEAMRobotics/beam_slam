cmake_minimum_required(VERSION 2.8.3)
project(bs_publishers)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


set(CMAKE_BUILD_TYPE "release")

find_package(
  beam REQUIRED 
  COMPONENTS 
  utils
)

set(catkin_build_depends
  fuse_core
  fuse_variables
  geometry_msgs
  nav_msgs
  pluginlib
  roscpp
  bs_common
  bs_constraints
)


find_package(
  catkin REQUIRED 
  COMPONENTS
    ${catkin_build_depends}
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${catkin_build_depends}
)

###########
## Build ##
###########

# bs_publishers library
add_library(
  ${PROJECT_NAME}
  src/path_3d_publisher.cpp
  src/odometry_3d_publisher.cpp
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
    ${beam_INCLUDE_DIRS}
)

target_link_libraries(
  ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    beam::utils
)
