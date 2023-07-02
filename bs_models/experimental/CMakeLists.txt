add_library(
  ${PROJECT_NAME}_experimental
  ## sensor models
  src/lidar_aggregation.cpp
  src/lidar_feature_extractor.cpp
  ## lidar helpers
  src/lib/lidar/lidar_aggregator.cpp
)

target_include_directories(${PROJECT_NAME}_experimental
  PUBLIC
    include
    ../include
)

target_link_libraries(${PROJECT_NAME}_experimental ${PROJECT_NAME})