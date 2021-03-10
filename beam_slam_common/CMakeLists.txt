cmake_minimum_required(VERSION 2.8.3)
project(beam_slam_common)

set(build_depends
  roscpp
  )

find_package(tf REQUIRED)
find_package(beam REQUIRED COMPONENTS utils)

find_package(
  catkin REQUIRED 
  COMPONENTS
    ${build_depends}
    fuse_variables
    fuse_constraints
  )

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${build_depends}
    fuse_variables
    fuse_constraints
)

###########
## Build ##
###########

add_compile_options(-std=c++14)

# library
add_library(${PROJECT_NAME}
  src/beam_parameters/params.cpp
  src/beam_parameters/models/model_params.cpp
  src/beam_parameters/publishers/publisher_params.cpp
  src/beam_common/common.cpp
  src/beam_common/pose_lookup.cpp
  )
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
  )
target_include_directories(${PROJECT_NAME}
  PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
  )
target_link_libraries(
  ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${tf_LIBRARIES}
    beam::utils
  )
