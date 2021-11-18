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

find_package(
  catkin REQUIRED
  COMPONENTS
    bs_models
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS 
    bs_models
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_library(
  ${PROJECT_NAME}
  src/placeholder.cpp
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    beam::beam
)

## Add executables
add_executable(${PROJECT_NAME}_global_map_refinement_main
	src/global_map_refinement_main.cpp
)
add_dependencies(${PROJECT_NAME}_global_map_refinement_main
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(${PROJECT_NAME}_global_map_refinement_main
  PUBLIC
    ${PROJECT_NAME}
)
target_link_libraries(${PROJECT_NAME}_global_map_refinement_main
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)