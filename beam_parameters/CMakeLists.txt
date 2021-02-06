cmake_minimum_required(VERSION 2.8.3)
project(beam_parameters)

set(build_depends
  roscpp
  )

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

add_compile_options(-std=c++14)

# library
add_library(${PROJECT_NAME}
  src/params.cpp
  src/models/model_params.cpp
  src/publishers/publisher_params.cpp
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
  )
