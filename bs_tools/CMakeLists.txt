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

find_package(bs_models REQUIRED)

## Add executables
add_executable(${PROJECT_NAME}_global_map_refinement_main
	src/global_map_refinement_main.cpp
)
target_include_directories(${PROJECT_NAME}_global_map_refinement_main
  PUBLIC
  ${bs_models_INCLUDE_DIRS}
  )
target_link_libraries(${PROJECT_NAME}_global_map_refinement_main
  beam::utils
	gflags
)