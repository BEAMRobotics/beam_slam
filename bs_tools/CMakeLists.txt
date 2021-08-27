cmake_minimum_required(VERSION 3.14)
project(bs_tools)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(
  beam REQUIRED 
  COMPONENTS 
  utils
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

## Add executables
add_executable(${PROJECT_NAME}_global_map_refinement_main
	src/global_map_refinement_main.cpp
)
target_include_directories(${PROJECT_NAME}_global_map_refinement_main
  PUBLIC
  ${CERES_INCLUDE_DIRS}
  ${fuse_core_INCLUDE_DIRS}
  ${fuse_graphs_INCLUDE_DIRS}
  ${fuse_models_INCLUDE_DIRS}
  ${fuse_publishers_INCLUDE_DIRS}
  ${fuse_variables_INCLUDE_DIRS}
  ${fuse_constraints_INCLUDE_DIRS}
  ${bs_constraints_INCLUDE_DIRS}
  ${bs_common_INCLUDE_DIRS}
  ${bs_variables_INCLUDE_DIRS}
  ${bs_models_INCLUDE_DIRS}
  )
target_link_libraries(${PROJECT_NAME}_global_map_refinement_main
  ${CERES_LIBRARIES}  
  ${fuse_core_LIBRARIES}
  ${fuse_graphs_LIBRARIES}
  ${fuse_models_LIBRARIES}
  ${fuse_publishers_LIBRARIES}
  ${fuse_variables_LIBRARIES}
  ${fuse_constraints_LIBRARIES}
  ${bs_constraints_LIBRARIES}
  ${bs_common_LIBRARIES}
  ${bs_variables_LIBRARIES}
  ${bs_models_LIBRARIES}
  beam::utils
	gflags
)