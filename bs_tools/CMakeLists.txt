cmake_minimum_required(VERSION 3.14)
project(bs_tools)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(
  beam REQUIRED 
  COMPONENTS 
  utils
  filtering
)

find_package(
  catkin REQUIRED
)

find_package(Ceres REQUIRED)
find_package(gflags REQUIRED)
find_package(fuse_core REQUIRED)
find_package(fuse_publishers REQUIRED)
find_package(fuse_variables REQUIRED)
find_package(fuse_graphs REQUIRED)
find_package(fuse_models REQUIRED)
find_package(fuse_constraints REQUIRED)

find_package(bs_constraints REQUIRED)
find_package(bs_common REQUIRED)
find_package(bs_variables REQUIRED)
find_package(bs_models REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${bs_common_INCLUDE_DIRS}
  ${bs_models_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME}
  src/pointcloud_to_global_map.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${bs_models_LIBRARIES}
  ${bs_common_LIBRARIES}
  beam::utils
  beam::filtering
)

## Add executables
add_executable(${PROJECT_NAME}_global_map_refinement_main
	src/global_map_refinement_main.cpp
)
target_include_directories(${PROJECT_NAME}_global_map_refinement_main
  PUBLIC
  ${bs_constraints_INCLUDE_DIRS}
  ${bs_models_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_global_map_refinement_main
  ${bs_constraints_LIBRARIES}
  ${bs_common_LIBRARIES}
  ${bs_variables_LIBRARIES}
  ${bs_models_LIBRARIES}
  beam::utils
	gflags
)

add_executable(${PROJECT_NAME}_pointcloud_to_global_map_main
	src/pointcloud_to_global_map_main.cpp
)
target_include_directories(${PROJECT_NAME}_pointcloud_to_global_map_main
  PUBLIC
  include
)
target_link_libraries(${PROJECT_NAME}_pointcloud_to_global_map_main
  ${PROJECT_NAME}
	gflags
)