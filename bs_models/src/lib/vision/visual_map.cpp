#include <bs_models/vision/visual_map.h>

#include <beam_utils/math.h>

#include <bs_constraints/visual/visual_constraint.h>
#include <bs_constraints/visual/visual_constraint_fixed.h>

namespace bs_models { namespace vision {

VisualMap::VisualMap(std::shared_ptr<beam_calibration::CameraModel> cam_model)
    : cam_model_(cam_model) {}

beam::opt<Eigen::Matrix4d> VisualMap::GetCameraPose(const ros::Time& stamp) {
  if (!extrinsics_.GetT_CAMERA_BASELINK(T_cam_baselink_)) {
    ROS_ERROR("Unable to get baselink to camera transform.");
    return {};
  }

  fuse_variables::Position3DStamped::SharedPtr p = GetPosition(stamp);
  fuse_variables::Orientation3DStamped::SharedPtr q = GetOrientation(stamp);
  if (p && q) {
    Eigen::Vector3d position(p->data());
    Eigen::Quaterniond orientation(q->w(), q->x(), q->y(), q->z());
    Eigen::Matrix4d T_WORLD_BASELINK;
    beam::QuaternionAndTranslationToTransformMatrix(orientation, position, T_WORLD_BASELINK);

    // transform pose from baselink coord space to camera coord space
    Eigen::Matrix4d T_WORLD_CAMERA = T_WORLD_BASELINK * T_cam_baselink_.inverse();
    return T_WORLD_CAMERA;
  } else {
    return {};
  }
}

beam::opt<Eigen::Matrix4d> VisualMap::GetBaselinkPose(const ros::Time& stamp) {
  if (!extrinsics_.GetT_CAMERA_BASELINK(T_cam_baselink_)) {
    ROS_ERROR("Unable to get baselink to camera transform.");
    return {};
  }

  fuse_variables::Position3DStamped::SharedPtr p = GetPosition(stamp);
  fuse_variables::Orientation3DStamped::SharedPtr q = GetOrientation(stamp);
  if (p && q) {
    Eigen::Vector3d position(p->data());
    Eigen::Quaterniond orientation(q->w(), q->x(), q->y(), q->z());
    Eigen::Matrix4d T_WORLD_BASELINK;
    beam::QuaternionAndTranslationToTransformMatrix(orientation, position, T_WORLD_BASELINK);
    return T_WORLD_BASELINK;
  } else {
    return {};
  }
}

fuse_variables::Point3DLandmark::SharedPtr VisualMap::GetLandmark(uint64_t landmark_id) {
  fuse_variables::Point3DLandmark::SharedPtr landmark =
      fuse_variables::Point3DLandmark::make_shared();
  auto landmark_uuid = fuse_core::uuid::generate(landmark->type(), landmark_id);
  if (graph_) {
    try {
      *landmark =
          dynamic_cast<const fuse_variables::Point3DLandmark&>(graph_->getVariable(landmark_uuid));
      return landmark;
    } catch (const std::out_of_range& oor) {
      if (landmark_positions_.find(landmark_id) == landmark_positions_.end()) {
        return nullptr;
      } else {
        return landmark_positions_[landmark_id];
      }
    }
  } else if (landmark_positions_.find(landmark_id) != landmark_positions_.end()) {
    return landmark_positions_[landmark_id];
  }

  return nullptr;
}

fuse_variables::Point3DFixedLandmark::SharedPtr VisualMap::GetFixedLandmark(uint64_t landmark_id) {
  fuse_variables::Point3DFixedLandmark::SharedPtr landmark =
      fuse_variables::Point3DFixedLandmark::make_shared();
  auto landmark_uuid = fuse_core::uuid::generate(landmark->type(), landmark_id);
  if (graph_) {
    try {
      *landmark = dynamic_cast<const fuse_variables::Point3DFixedLandmark&>(
          graph_->getVariable(landmark_uuid));
      return landmark;
    } catch (const std::out_of_range& oor) {
      if (fixed_landmark_positions_.find(landmark_id) == fixed_landmark_positions_.end()) {
        return nullptr;
      } else {
        return fixed_landmark_positions_[landmark_id];
      }
    }
  } else if (fixed_landmark_positions_.find(landmark_id) != fixed_landmark_positions_.end()) {
    return fixed_landmark_positions_[landmark_id];
  }

  return nullptr;
}

fuse_variables::Orientation3DStamped::SharedPtr VisualMap::GetOrientation(const ros::Time& stamp) {
  fuse_variables::Orientation3DStamped::SharedPtr corr_orientation =
      fuse_variables::Orientation3DStamped::make_shared();
  auto corr_orientation_uuid =
      fuse_core::uuid::generate(corr_orientation->type(), stamp, fuse_core::uuid::NIL);
  if (graph_) {
    try {
      *corr_orientation = dynamic_cast<const fuse_variables::Orientation3DStamped&>(
          graph_->getVariable(corr_orientation_uuid));
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

  return nullptr;
}

fuse_variables::Position3DStamped::SharedPtr VisualMap::GetPosition(const ros::Time& stamp) {
  fuse_variables::Position3DStamped::SharedPtr corr_position =
      fuse_variables::Position3DStamped::make_shared();
  auto corr_position_uuid =
      fuse_core::uuid::generate(corr_position->type(), stamp, fuse_core::uuid::NIL);
  if (graph_) {
    try {
      *corr_position = dynamic_cast<const fuse_variables::Position3DStamped&>(
          graph_->getVariable(corr_position_uuid));
      return corr_position;
    } catch (const std::out_of_range& oor) {
      if (positions_.find(stamp.toNSec()) == positions_.end()) {
        return nullptr;
      } else {
        return positions_[stamp.toNSec()];
      }
    }
  } else if (positions_.find(stamp.toNSec()) != positions_.end()) {
    return positions_[stamp.toNSec()];
  }

  return nullptr;
}

void VisualMap::AddCameraPose(const Eigen::Matrix4d& T_WORLD_CAMERA, const ros::Time& stamp,
                              fuse_core::Transaction::SharedPtr transaction) {
  if (!extrinsics_.GetT_CAMERA_BASELINK(T_cam_baselink_)) {
    ROS_ERROR("Unable to get baselink to camera transform.");
    return;
  }

  // transform pose into baselink coord space
  Eigen::Matrix4d T_WORLD_BASELINK = T_WORLD_CAMERA * T_cam_baselink_;
  Eigen::Quaterniond q;
  Eigen::Vector3d p;
  beam::TransformMatrixToQuaternionAndTranslation(T_WORLD_BASELINK, q, p);

  // add position variable
  AddPosition(p, stamp, transaction);

  // add orientation variable
  AddOrientation(q, stamp, transaction);
}

void VisualMap::AddBaselinkPose(const Eigen::Matrix4d& T_WORLD_BASELINK, const ros::Time& stamp,
                                fuse_core::Transaction::SharedPtr transaction) {
  Eigen::Quaterniond q;
  Eigen::Vector3d p;
  beam::TransformMatrixToQuaternionAndTranslation(T_WORLD_BASELINK, q, p);

  // add position variable
  AddPosition(p, stamp, transaction);

  // add orientation variable
  AddOrientation(q, stamp, transaction);
}

void VisualMap::AddOrientation(const Eigen::Quaterniond& q_WORLD_BASELINK, const ros::Time& stamp,
                               fuse_core::Transaction::SharedPtr transaction) {
  // cosntruct orientation variable
  fuse_variables::Orientation3DStamped::SharedPtr orientation =
      fuse_variables::Orientation3DStamped::make_shared(stamp);
  orientation->w() = q_WORLD_BASELINK.w();
  orientation->x() = q_WORLD_BASELINK.x();
  orientation->y() = q_WORLD_BASELINK.y();
  orientation->z() = q_WORLD_BASELINK.z();

  // add fuse orientation variable
  AddOrientation(orientation, transaction);
}

void VisualMap::AddPosition(const Eigen::Vector3d& p_WORLD_BASELINK, const ros::Time& stamp,
                            fuse_core::Transaction::SharedPtr transaction) {
  // construct position variable
  fuse_variables::Position3DStamped::SharedPtr position =
      fuse_variables::Position3DStamped::make_shared(stamp);
  position->x() = p_WORLD_BASELINK[0];
  position->y() = p_WORLD_BASELINK[1];
  position->z() = p_WORLD_BASELINK[2];

  // add fuse position variable
  AddPosition(position, transaction);
}

void VisualMap::AddLandmark(const Eigen::Vector3d& position, uint64_t id,
                            fuse_core::Transaction::SharedPtr transaction) {
  // construct landmark variable
  fuse_variables::Point3DLandmark::SharedPtr landmark =
      fuse_variables::Point3DLandmark::make_shared(id);
  landmark->x() = position[0];
  landmark->y() = position[1];
  landmark->z() = position[2];

  // add fuse landmark variable
  AddLandmark(landmark, transaction);
}

void VisualMap::AddFixedLandmark(const Eigen::Vector3d& position, uint64_t id,
                                 fuse_core::Transaction::SharedPtr transaction) {
  // construct landmark variable
  fuse_variables::Point3DFixedLandmark::SharedPtr landmark =
      fuse_variables::Point3DFixedLandmark::make_shared(id);
  landmark->x() = position[0];
  landmark->y() = position[1];
  landmark->z() = position[2];

  // add fuse landmark variable
  AddFixedLandmark(landmark, transaction);
}

void VisualMap::AddOrientation(fuse_variables::Orientation3DStamped::SharedPtr orientation,
                               fuse_core::Transaction::SharedPtr transaction) {
  // add to transaction
  transaction->addVariable(orientation);
  orientations_[orientation->stamp().toNSec()] = orientation;
}

void VisualMap::AddPosition(fuse_variables::Position3DStamped::SharedPtr position,
                            fuse_core::Transaction::SharedPtr transaction) {
  // add to transaction
  transaction->addVariable(position);
  positions_[position->stamp().toNSec()] = position;
}

void VisualMap::AddLandmark(fuse_variables::Point3DLandmark::SharedPtr landmark,
                            fuse_core::Transaction::SharedPtr transaction) {
  // add to transaction
  transaction->addVariable(landmark);
  landmark_positions_[landmark->id()] = landmark;
}

void VisualMap::AddFixedLandmark(fuse_variables::Point3DFixedLandmark::SharedPtr landmark,
                                 fuse_core::Transaction::SharedPtr transaction) {
  // add to transaction
  transaction->addVariable(landmark);
  fixed_landmark_positions_[landmark->id()] = landmark;
}

bool VisualMap::AddVisualConstraint(const ros::Time& stamp, uint64_t lm_id,
                                    const Eigen::Vector2d& pixel,
                                    fuse_core::Transaction::SharedPtr transaction) {
  if (!extrinsics_.GetT_CAMERA_BASELINK(T_cam_baselink_)) {
    ROS_ERROR("Unable to get baselink to camera transform.");
    return false;
  }

  // get landmark (fixed or not fixed)
  fuse_variables::Point3DLandmark::SharedPtr lm = GetLandmark(lm_id);
  fuse_variables::Point3DFixedLandmark::SharedPtr lm_fixed = GetFixedLandmark(lm_id);

  // get robot pose
  fuse_variables::Position3DStamped::SharedPtr position = GetPosition(stamp);
  fuse_variables::Orientation3DStamped::SharedPtr orientation = GetOrientation(stamp);
  if (position && orientation) {
    try {
      if (lm) {
        // add normal visual constraint
        fuse_constraints::VisualConstraint::SharedPtr vis_constraint =
            fuse_constraints::VisualConstraint::make_shared(source_, *orientation, *position, *lm,
                                                            pixel, T_cam_baselink_, cam_model_,
                                                            "HUBER", "VANILLA");
        transaction->addConstraint(vis_constraint);
        return true;
      } else if (lm_fixed) {
        // add fixed visual constraint
        fuse_constraints::VisualConstraintFixed::SharedPtr vis_constraint =
            fuse_constraints::VisualConstraintFixed::make_shared(source_, *orientation, *position,
                                                                 *lm_fixed, pixel, T_cam_baselink_,
                                                                 cam_model_, "HUBER", "VANILLA");
        transaction->addConstraint(vis_constraint);
        return true;
      }
    } catch (const std::logic_error& le) {}
  }
  return false;
}

fuse_core::UUID VisualMap::GetLandmarkUUID(uint64_t landmark_id) {
  fuse_variables::Point3DLandmark::SharedPtr landmark =
      fuse_variables::Point3DLandmark::make_shared();
  auto landmark_uuid = fuse_core::uuid::generate(landmark->type(), landmark_id);
  return landmark_uuid;
}

fuse_core::UUID VisualMap::GetFixedLandmarkUUID(uint64_t landmark_id) {
  fuse_variables::Point3DFixedLandmark::SharedPtr landmark =
      fuse_variables::Point3DFixedLandmark::make_shared();
  auto landmark_uuid = fuse_core::uuid::generate(landmark->type(), landmark_id);
  return landmark_uuid;
}

fuse_core::UUID VisualMap::GetPositionUUID(const ros::Time& stamp) {
  fuse_variables::Position3DStamped::SharedPtr corr_position =
      fuse_variables::Position3DStamped::make_shared();
  auto corr_position_uuid =
      fuse_core::uuid::generate(corr_position->type(), stamp, fuse_core::uuid::NIL);
  return corr_position_uuid;
}

fuse_core::UUID VisualMap::GetOrientationUUID(const ros::Time& stamp) {
  fuse_variables::Orientation3DStamped::SharedPtr corr_orientation =
      fuse_variables::Orientation3DStamped::make_shared();
  auto corr_orientation_uuid =
      fuse_core::uuid::generate(corr_orientation->type(), stamp, fuse_core::uuid::NIL);
  return corr_orientation_uuid;
}

bool VisualMap::PoseExists(const ros::Time& stamp) {
  if (!graph_) { return false; }
  if (graph_->variableExists(GetOrientationUUID(stamp)) &&
      graph_->variableExists(GetPositionUUID(stamp))) {
    return true;
  }
  return false;
}

bool VisualMap::LandmarkExists(uint64_t landmark_id) {
  if (!graph_) { return false; }
  if (graph_->variableExists(GetLandmarkUUID(landmark_id))) { return true; }
  return false;
}

bool VisualMap::FixedLandmarkExists(uint64_t landmark_id) {
  if (!graph_) { return false; }
  if (graph_->variableExists(GetFixedLandmarkUUID(landmark_id))) { return true; }
  return false;
}

void VisualMap::UpdateGraph(fuse_core::Graph::ConstSharedPtr graph_msg) {
  graph_ = graph_msg;

  // remove duplicate orientations
  auto orientation_itr = orientations_.begin();
  while (orientation_itr != orientations_.end()) {
    auto uuid = GetOrientationUUID(beam::NSecToRos(orientation_itr->first));
    if (graph_->variableExists(uuid)) {
      orientations_.erase(orientation_itr++);
    } else {
      ++orientation_itr;
    }
  }

  // remove duplicate positions
  auto position_itr = positions_.begin();
  while (position_itr != positions_.end()) {
    auto uuid = GetPositionUUID(beam::NSecToRos(position_itr->first));
    if (graph_->variableExists(uuid)) {
      positions_.erase(position_itr++);
    } else {
      ++position_itr;
    }
  }

  // remove duplicate landmarks
  auto landmark_itr = landmark_positions_.begin();
  while (landmark_itr != landmark_positions_.end()) {
    auto uuid = GetLandmarkUUID(landmark_itr->first);
    if (graph_->variableExists(uuid)) {
      landmark_positions_.erase(landmark_itr++);
    } else {
      ++landmark_itr;
    }
  }

  // remove duplicate landmarks
  auto fixed_landmark_itr = fixed_landmark_positions_.begin();
  while (fixed_landmark_itr != fixed_landmark_positions_.end()) {
    auto uuid = GetFixedLandmarkUUID(fixed_landmark_itr->first);
    if (graph_->variableExists(uuid)) {
      fixed_landmark_positions_.erase(fixed_landmark_itr++);
    } else {
      ++fixed_landmark_itr;
    }
  }
}

void VisualMap::Clear() {
  orientations_.clear();
  positions_.clear();
  landmark_positions_.clear();
  fixed_landmark_positions_.clear();
  if (graph_) { graph_ = nullptr; }
}

}} // namespace bs_models::vision