cmake_minimum_required(VERSION 3.14)
project(bs_optimizers)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_BUILD_TYPE "release")

find_package(
  beam REQUIRED 
  COMPONENTS 
  utils
)

set(build_depends
  diagnostic_updater
  fuse_optimizers
  fuse_constraints
  fuse_core
  fuse_graphs
  fuse_variables
  pluginlib
  roscpp
  std_srvs
  bs_constraints
  bs_common
)

find_package(OpenMP)

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
add_compile_options(-Wall -Werror)

## fuse_optimizers library
add_library(${PROJECT_NAME}
  src/fixed_lag_smoother.cpp
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
  ${OpenMP_CXX_LIBRARIES}
  beam::utils
)
set_target_properties(${PROJECT_NAME}
  PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
)
target_compile_options(${PROJECT_NAME}
  PRIVATE ${OpenMP_CXX_FLAGS}
)

## fixed_lag_smoother node
add_executable(fixed_lag_smoother_node
  src/fixed_lag_smoother_node.cpp
)
add_dependencies(fixed_lag_smoother_node
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(fixed_lag_smoother_node
  PRIVATE
    include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(fixed_lag_smoother_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  beam::utils
)
set_target_properties(fixed_lag_smoother_node
  PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
)
