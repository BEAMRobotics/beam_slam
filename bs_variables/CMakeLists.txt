cmake_minimum_required(VERSION 3.14)
project(bs_variables)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_BUILD_TYPE "release")

set(build_depends
  fuse_core
  pluginlib
  roscpp
)

find_package(catkin REQUIRED COMPONENTS
  ${build_depends}
)
find_package(Ceres REQUIRED)

find_package(
  catkin REQUIRED
  COMPONENTS
    fuse_variables)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${build_depends}
  DEPENDS
    CERES
)

###########
## Build ##
###########
add_compile_options(-Wall -Werror)

## bs_variables library
add_library(${PROJECT_NAME}
  src/accel_bias_3d_stamped.cpp
  src/gyro_bias_3d_stamped.cpp
  src/inverse_depth_landmark.cpp
  src/point_3d_landmark.cpp
  src/position_3d.cpp
  src/orientation_3d.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
)
set_target_properties(${PROJECT_NAME}
  PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED YES
)
