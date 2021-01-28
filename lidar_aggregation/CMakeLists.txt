cmake_minimum_required(VERSION 2.8.3)
project(lidar_aggregation)

## c++11 feature
SET(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_FLAGS "-std=gnu++11 -Wall -Wextra ${CMAKE_CXX_FLAGS}")
add_definitions(-DBOOST_LOG_DYN_LINK)
set(catkin_deps roscpp roslib nodelet std_msgs nav_msgs sensor_msgs)

find_package(catkin REQUIRED ${catkin_deps})
find_package(Boost COMPONENTS REQUIRED log_setup log)
find_package(beam REQUIRED utils)
find_package(nodelet REQUIRED)
find_package(roscpp REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS ${catkin_deps}
    DEPENDS Boost
    LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_nodelet
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${nodelet_INCLUDE_DIRS}
    ${roscpp_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/lidar_aggregators/lidar_aggregator_base.cpp
    src/lidar_aggregators/end_time_lidar_aggregator.cpp
    #src/lidar_aggregators/center_time_lidar_aggregator.cpp
)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    beam::utils
)

#### ROS Nodelets
add_library(${PROJECT_NAME}_nodelet
    src/${PROJECT_NAME}_nodelet.cpp
)
target_link_libraries(${PROJECT_NAME}_nodelet
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
    ${Boost_LIBRARIES}
    ${nodelet_LIBRARIES}
    ${roscpp_LIBRARIES}
    beam::utils
)
add_library(${PROJECT_NAME}_time_publisher_nodelet
    src/${PROJECT_NAME}_time_publisher_nodelet.cpp
)
target_link_libraries(${PROJECT_NAME}_time_publisher_nodelet
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
    ${Boost_LIBRARIES}
    ${nodelet_LIBRARIES}
    ${roscpp_LIBRARIES}
)

#### ROS Nodes
add_executable(${PROJECT_NAME}_node
    src/${PROJECT_NAME}_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${nodelet_LIBRARIES}
    ${roscpp_LIBRARIES}
    beam::utils
)
add_executable(${PROJECT_NAME}_time_publisher_node
    src/${PROJECT_NAME}_time_publisher_node.cpp
)
target_link_libraries(${PROJECT_NAME}_time_publisher_node
    ${catkin_LIBRARIES}
    ${nodelet_LIBRARIES}
    ${roscpp_LIBRARIES}
)