cmake_minimum_required(VERSION 3.14)
project(bs_tools)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_BUILD_TYPE "release")

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

## Add executables
add_executable(${PROJECT_NAME}_global_map_refinement_main
	src/global_map_refinement_main.cpp
)
target_include_directories(${PROJECT_NAME}_global_map_refinement_main
  PUBLIC
    # ${PROJECT_NAME}
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_global_map_refinement_main
  # ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  beam::utils
)

add_executable(calibration_viewer
  src/calibration_viewer_node.cpp
)
target_include_directories(calibration_viewer
  PUBLIC
    # ${PROJECT_NAME}
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(calibration_viewer
  # ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  beam::utils
  beam::calibration
  beam::cv
)

