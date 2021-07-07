#include <beam_constraints/camera_to_camera/visual_constraint.h>
#include <beam_models/camera_to_camera/visual_map.h>
#include <beam_utils/math.h>

namespace beam_models { namespace camera_to_camera {

VisualMap::VisualMap(std::shared_ptr<beam_calibration::CameraModel> cam_model,
                     const Eigen::Matrix4d& T_imu_cam,
                     const std::string& source)
    : cam_model_(cam_model), T_imu_cam_(T_imu_cam), source_(source) {}

VisualMap::VisualMap(std::shared_ptr<beam_calibration::CameraModel> cam_model,
                     fuse_core::Graph::SharedPtr local_graph,
                     const Eigen::Matrix4d& T_imu_cam,
                     const std::string& source)
    : cam_model_(cam_model),
      local_graph_(local_graph),
      T_imu_cam_(T_imu_cam),
      source_(source) {}

beam::opt<Eigen::Matrix4d> VisualMap::GetPose(const ros::Time& stamp) {
  fuse_variables::Position3DStamped::SharedPtr p = GetPosition(stamp);
  fuse_variables::Orientation3DStamped::SharedPtr q = GetOrientation(stamp);
  if (p && q) {
    Eigen::Vector3d position(p->data());
    Eigen::Quaterniond orientation(q->w(), q->x(), q->y(), q->z());
    Eigen::Matrix4d T_WORLD_IMU;
    beam::QuaternionAndTranslationToTransformMatrix(orientation, position,
                                                    T_WORLD_IMU);
    // transform pose from imu coord space to camera coord space
    Eigen::Matrix4d T_WORLD_CAMERA = T_WORLD_IMU * T_imu_cam_;
    return T_WORLD_CAMERA;
  } else {
    return {};
  }
}

fuse_variables::Position3D::SharedPtr
    VisualMap::GetLandmark(uint64_t landmark_id) {
  fuse_variables::Position3D::SharedPtr landmark =
      fuse_variables::Position3D::make_shared();
  auto landmark_uuid = fuse_core::uuid::generate(
      landmark->type(), std::to_string(landmark_id).c_str());
  // first check the graph for the variable if its initialized
  if (!local_graph_) {
    if (graph_initialized) {
      try {
        *landmark = dynamic_cast<const fuse_variables::Position3D&>(
            graph_->getVariable(landmark_uuid));
        landmark_positions_.erase(landmark_id);
        return landmark;
      } catch (const std::out_of_range& oor) {
        if (landmark_positions_.find(landmark_id) ==
            landmark_positions_.end()) {
          return nullptr;
        } else {
          return landmark_positions_[landmark_id];
        }
      }
    }
    // if its not initialized check local maps
    if (landmark_positions_.find(landmark_id) != landmark_positions_.end()) {
      return landmark_positions_[landmark_id];
    }
  } else {
    try {
      *landmark = dynamic_cast<const fuse_variables::Position3D&>(
          local_graph_->getVariable(landmark_uuid));
      return landmark;
    } catch (const std::out_of_range& oor) {}
  }
  return nullptr;
}

void VisualMap::AddPose(const Eigen::Matrix4d& T_WORLD_CAMERA,
                        const ros::Time& cur_time,
                        fuse_core::Transaction::SharedPtr transaction) {
  // transform pose into imu coord space
  Eigen::Matrix4d T_WORLD_IMU = T_WORLD_CAMERA * T_imu_cam_.inverse();
  Eigen::Quaterniond q;
  Eigen::Vector3d p;
  beam::TransformMatrixToQuaternionAndTranslation(T_WORLD_IMU, q, p);
  // add orientation
  fuse_variables::Orientation3DStamped::SharedPtr orientation =
      fuse_variables::Orientation3DStamped::make_shared(cur_time);
  orientation->w() = q.w();
  orientation->x() = q.x();
  orientation->y() = q.y();
  orientation->z() = q.z();
  // add position
  fuse_variables::Position3DStamped::SharedPtr position =
      fuse_variables::Position3DStamped::make_shared(cur_time);
  position->x() = p[0];
  position->y() = p[1];
  position->z() = p[2];

  if (transaction) {
    transaction->addVariable(orientation);
    orientations_[cur_time.toNSec()] = orientation;
    transaction->addVariable(position);
    positions_[cur_time.toNSec()] = position;
  } else if (local_graph_) {
    local_graph_->addVariable(position);
    local_graph_->addVariable(orientation);
  } else {
    ROS_WARN("Must input local graph or transaction.");
  }
}

void VisualMap::AddOrientation(const Eigen::Quaterniond& q_WORLD_IMU,
                               const ros::Time& stamp,
                               fuse_core::Transaction::SharedPtr transaction) {
  fuse_variables::Orientation3DStamped::SharedPtr orientation =
      fuse_variables::Orientation3DStamped::make_shared(stamp);
  orientation->w() = q_WORLD_IMU.w();
  orientation->x() = q_WORLD_IMU.x();
  orientation->y() = q_WORLD_IMU.y();
  orientation->z() = q_WORLD_IMU.z();
  if (transaction) {
    transaction->addVariable(orientation);
    orientations_[stamp.toNSec()] = orientation;
  } else if (local_graph_) {
    local_graph_->addVariable(orientation);
  } else {
    ROS_WARN("Must input local graph or transaction.");
  }
}

void VisualMap::AddPosition(const Eigen::Vector3d& p_WORLD_IMU,
                            const ros::Time& stamp,
                            fuse_core::Transaction::SharedPtr transaction) {
  fuse_variables::Position3DStamped::SharedPtr position =
      fuse_variables::Position3DStamped::make_shared(stamp);
  position->x() = p_WORLD_IMU[0];
  position->y() = p_WORLD_IMU[1];
  position->z() = p_WORLD_IMU[2];
  if (transaction) {
    transaction->addVariable(position);
    positions_[stamp.toNSec()] = position;
  } else if (local_graph_) {
    local_graph_->addVariable(position);
  } else {
    ROS_WARN("Must input local graph or transaction.");
  }
}

void VisualMap::AddLandmark(const Eigen::Vector3d& position, uint64_t id,
                            fuse_core::Transaction::SharedPtr transaction) {
  fuse_variables::Position3D::SharedPtr landmark =
      fuse_variables::Position3D::make_shared(id);
  landmark->x() = position[0];
  landmark->y() = position[1];
  landmark->z() = position[2];

  if (transaction) {
    transaction->addVariable(landmark);
    landmark_positions_[id] = landmark;
  } else if (local_graph_) {
    local_graph_->addVariable(landmark);
  } else {
    ROS_WARN("Must input local graph or transaction.");
  }
}

void VisualMap::AddConstraint(const ros::Time& img_time, uint64_t lm_id,
                              const Eigen::Vector2d& pixel,
                              fuse_core::Transaction::SharedPtr transaction) {
  fuse_variables::Position3D::SharedPtr lm = GetLandmark(lm_id);
  fuse_variables::Position3DStamped::SharedPtr position = GetPosition(img_time);
  fuse_variables::Orientation3DStamped::SharedPtr orientation =
      GetOrientation(img_time);
  if (position && orientation && lm) {
    fuse_constraints::VisualConstraint::SharedPtr vis_constraint =
        fuse_constraints::VisualConstraint::make_shared(source_, *orientation,
                                                        *position, *lm, pixel,
                                                        T_imu_cam_, cam_model_);
    if (transaction) {
      transaction->addConstraint(vis_constraint);
    } else if (local_graph_) {
      local_graph_->addConstraint(vis_constraint);
    } else {
      ROS_WARN("Must input local graph or transaction.");
    }
  }
}

fuse_variables::Orientation3DStamped::SharedPtr
    VisualMap::GetOrientation(const ros::Time& stamp) {
  fuse_variables::Orientation3DStamped::SharedPtr corr_orientation =
      fuse_variables::Orientation3DStamped::make_shared();
  auto corr_orientation_uuid = fuse_core::uuid::generate(
      corr_orientation->type(), stamp, fuse_core::uuid::NIL);
  // first check the graph for the variable if its initialized
  if (!local_graph_) {
    if (graph_initialized) {
      try {
        *corr_orientation =
            dynamic_cast<const fuse_variables::Orientation3DStamped&>(
                graph_->getVariable(corr_orientation_uuid));
        orientations_.erase(stamp.toSec());
        return corr_orientation;
      } catch (const std::out_of_range& oor) {
        if (orientations_.find(stamp.toNSec()) == orientations_.end()) {
          return nullptr;
        } else {
          return orientations_[stamp.toNSec()];
        }
      }
    } else if (orientations_.find(stamp.toNSec()) != orientations_.end()) {
      return orientations_[stamp.toNSec()];
    }
  } else {
    try {
      *corr_orientation =
          dynamic_cast<const fuse_variables::Orientation3DStamped&>(
              local_graph_->getVariable(corr_orientation_uuid));
      return corr_orientation;
    } catch (const std::out_of_range& oor) {}
  }
  return nullptr;
}

fuse_variables::Position3DStamped::SharedPtr
    VisualMap::GetPosition(const ros::Time& stamp) {
  fuse_variables::Position3DStamped::SharedPtr corr_position =
      fuse_variables::Position3DStamped::make_shared();
  auto corr_position_uuid = fuse_core::uuid::generate(
      corr_position->type(), stamp, fuse_core::uuid::NIL);
  // first check the graph for the variable if its initialized
  if (!local_graph_) {
    if (graph_initialized) {
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
  } else {
    try {
      *corr_position = dynamic_cast<const fuse_variables::Position3DStamped&>(
          local_graph_->getVariable(corr_position_uuid));
      return corr_position;
    } catch (const std::out_of_range& oor) {}
  }
  return nullptr;
}

void VisualMap::UpdateGraph(fuse_core::Graph::ConstSharedPtr graph_msg) {
  // clear temp maps
  orientations_.clear();
  positions_.clear();
  landmark_positions_.clear();
  graph_ = std::move(graph_msg);
  if (!graph_initialized) graph_initialized = true;
}

}} // namespace beam_models::camera_to_camera