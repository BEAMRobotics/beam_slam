#include <beam_models/frame_initializers/graph_frame_initializer.h>

namespace beam_models { namespace frame_initializers {

GraphFrameInitializer::GraphFrameInitializer() {}

bool GraphFrameInitializer::GetEstimatedPose(const ros::Time& time,
                                             Eigen::Matrix4d& T_WORLD_SENSOR) {
  fuse_variables::Orientation3DStamped::SharedPtr R_WORLD_SENSOR =
      this->GetOrientation(time);
  fuse_variables::Position3DStamped::SharedPtr t_WORLD_SENSOR =
      this->GetPosition(time);
  if (R_WORLD_SENSOR && t_WORLD_SENSOR) {
    // success, pose is in graph
    Eigen::Quaterniond q(R_WORLD_SENSOR->w(), R_WORLD_SENSOR->x(),
                         R_WORLD_SENSOR->y(), R_WORLD_SENSOR->z());
    Eigen::Vector3d t(t_WORLD_SENSOR->x(), t_WORLD_SENSOR->y(),
                      t_WORLD_SENSOR->z());
    beam::QuaternionAndTranslationToTransformMatrix(q, t, T_WORLD_SENSOR);
    return true;
  } else {
    // fail, pose not in graph
    if (this->imu_preint_) {
      ros::Time imu_window_start = this->imu_preint_->GetImuState().Stamp();
      // requested pose not in graph, or in future
      if (time < imu_window_start) {
        return false;
      } else {
        T_WORLD_SENSOR = this->imu_preint_->GetPose(time);
        return true;
      }
    }
  }
}

fuse_variables::Orientation3DStamped::SharedPtr
    GraphFrameInitializer::GetOrientation(const ros::Time& stamp) {
  std::string orientation_3d_stamped_type =
      "fuse_variables::Orientation3DStamped";
  fuse_variables::Orientation3DStamped::SharedPtr corr_orientation =
      fuse_variables::Orientation3DStamped::make_shared();
  // first check the graph for the variable if its initialized
  if (graph_) {
    auto corr_orientation_uuid = fuse_core::uuid::generate(
        orientation_3d_stamped_type, stamp, fuse_core::uuid::NIL);
    try {
      *corr_orientation =
          dynamic_cast<const fuse_variables::Orientation3DStamped&>(
              graph_->getVariable(corr_orientation_uuid));
      orientations_.erase(stamp.toSec());
    } catch (const std::out_of_range& oor) {
      if (orientations_.find(stamp.toNSec()) == orientations_.end()) {
        return nullptr;
      } else {
        return orientations_[stamp.toNSec()];
      }
    }
  }
  // if its not initalized check local maps
  if (orientations_.find(stamp.toNSec()) != orientations_.end()) {
    return orientations_[stamp.toNSec()];
  }
  return nullptr;
}

fuse_variables::Position3DStamped::SharedPtr
    GraphFrameInitializer::GetPosition(const ros::Time& stamp) {
  std::string position_3d_stamped_type = "fuse_variables::Position3DStamped";
  fuse_variables::Position3DStamped::SharedPtr corr_position =
      fuse_variables::Position3DStamped::make_shared();
  // first check the graph for the variable if its initialized
  if (graph_) {
    auto corr_position_uuid = fuse_core::uuid::generate(
        position_3d_stamped_type, stamp, fuse_core::uuid::NIL);
    try {
      *corr_position = dynamic_cast<const fuse_variables::Position3DStamped&>(
          graph_->getVariable(corr_position_uuid));
      positions_.erase(stamp.toSec());
      return corr_position;
    } catch (const std::out_of_range& oor) {
      if (positions_.find(stamp.toNSec()) == positions_.end()) {
        return nullptr;
      } else {
        return positions_[stamp.toNSec()];
      }
    }
  }
  // if its not initalized check local maps
  if (positions_.find(stamp.toNSec()) != positions_.end()) {
    return positions_[stamp.toNSec()];
  }
  return nullptr;
}

void GraphFrameInitializer::AddOrientation(
    const ros::Time& stamp,
    const fuse_variables::Orientation3DStamped::SharedPtr& R_WORLD_SENSOR) {
  orientations_[stamp.toNSec()] = R_WORLD_SENSOR;
}

void GraphFrameInitializer::AddPosition(
    const ros::Time& stamp,
    const fuse_variables::Position3DStamped::SharedPtr& t_WORLD_SENSOR) {
  positions_[stamp.toNSec()] = t_WORLD_SENSOR;
}

void GraphFrameInitializer::SetGraph(
    const fuse_core::Graph::ConstSharedPtr& graph) {
  this->graph_ = std::move(graph);
  orientations_.clear();
  positions_.clear();
}

void GraphFrameInitializer::SetIMUPreintegrator(
    std::shared_ptr<beam_models::frame_to_frame::ImuPreintegration>
        imu_preint) {
  this->imu_preint_ = imu_preint;
}

}} // namespace beam_models::frame_initializers