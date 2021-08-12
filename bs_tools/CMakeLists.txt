cmake_minimum_required(VERSION 3.14)
project(bs_tools)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(
  beam REQUIRED 
  COMPONENTS 
  calibration
  mapping
  filtering 
  matching
  cv
  utils
  )

find_package(gflags REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV 4.5.2 REQUIRED COMPONENTS)

find_package(fuse_core REQUIRED)
find_package(fuse_variables REQUIRED)
find_package(fuse_graphs REQUIRED)
find_package(fuse_constraints REQUIRED)

find_package(bs_constraints REQUIRED)
find_package(bs_common REQUIRED)
find_package(bs_variables REQUIRED)
find_package(bs_models REQUIRED)

###########
## Build ##
###########
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${fuse_graphs_INCLUDE_DIRS}
  ${fuse_models_INCLUDE_DIRS}
  ${fuse_variables_INCLUDE_DIRS}
  ${fuse_constraints_INCLUDE_DIRS}
  ${bs_constraints_INCLUDE_DIRS}
  ${bs_common_INCLUDE_DIRS}
  ${bs_variables_INCLUDE_DIRS}
  ${bs_models_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(
  ${PROJECT_NAME}
  src/global_map_refinement.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
  ${PROJECT_NAME}
    ${Boost_LIBRARIES}
    ${CERES_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${fuse_graphs_LIBRARIES}
    ${fuse_variables_LIBRARIES}
    ${fuse_constraints_LIBRARIES}
    ${bs_constraints_LIBRARIES}
    ${bs_common_LIBRARIES}
    ${bs_variables_LIBRARIES}
    ${bs_models_LIBRARIES}
    beam::utils
    beam::matching
    beam::filtering
    beam::mapping
    beam::calibration
    beam::cv
)

## Add executables
add_executable(${PROJECT_NAME}_global_map_refinement_main
	src/global_map_refinement_main.cpp
)
target_link_libraries(${PROJECT_NAME}_global_map_refinement_main
	${PROJECT_NAME}
  beam::utils
	gflags
)
