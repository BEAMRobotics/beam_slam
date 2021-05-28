#include <gtest/gtest.h>
#include <fuse_graphs/hash_graph.h>
#include <fuse_core/uuid.h>
#include <fuse_core/variable.h>

#include <beam_models/frame_to_frame/graph_frame_initializer.h>



TEST(GraphFrameInitializer) {
  // Manually create graph with 2 poses
  // create imu preint with 2nd pose as the start, and with some messages after populated
  // test getting either pose in the graph
  // test getting a pose in the future
  // test getting a pose no in graph nor in future
}