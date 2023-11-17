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
    cv
)

set(catkin_build_depends
    roscpp
    sensor_msgs
    bs_common
    bs_models
)

find_package(
  catkin REQUIRED
  COMPONENTS
    ${catkin_build_depends}
    message_generation
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS 
    ${catkin_build_depends}
    message_runtime
)

## Add executables
add_executable(${PROJECT_NAME}_global_map_refinement_main
	src/global_map_refinement_main.cpp
)
target_include_directories(${PROJECT_NAME}_global_map_refinement_main
  PUBLIC
    ${PROJECT_NAME}
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_global_map_refinement_main
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  beam::utils
)

add_executable(feature_tracker
  src/feature_tracker_node.cpp
)
target_include_directories(feature_tracker
  PUBLIC
    ${PROJECT_NAME}
)
target_link_libraries(feature_tracker
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  beam::utils
  beam::cv
)

add_executable(calibration_viewer
  src/calibration_viewer_node.cpp
)
target_include_directories(calibration_viewer
  PUBLIC
    ${PROJECT_NAME}
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(calibration_viewer
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  beam::utils
  beam::calibration
  beam::cv
)

