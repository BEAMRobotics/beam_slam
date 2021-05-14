#include <global_mapping/global_mapper.h>

#include <fuse_core/transaction.h>
#include <pluginlib/class_list_macros.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(global_mapping::GlobalMapper, fuse_core::SensorModel)

namespace global_mapping {

GlobalMapper::GlobalMapper()
    : fuse_core::AsyncSensorModel(1),
      device_id_(fuse_core::uuid::NIL),
      throttled_callback_(
          std::bind(&GlobalMapper::process, this, std::placeholders::_1)) {}

void GlobalMapper::process(const SlamChunk::ConstPtr& msg) {
  //
}

void GlobalMapper::onInit() {
  //
}

void GlobalMapper::onStart() {
  //
}

void GlobalMapper::onStop() {
  //
}

void GlobalMapper::onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph_msg) {
  //
}

}  // namespace global_mapping
