#include <bs_models/vision/visual_map.h>

#include <beam_utils/math.h>

#include <bs_constraints/visual/visual_constraint.h>
#include <bs_constraints/visual/visual_constraint_fixed.h>

namespace bs_models {
namespace vision {

VisualMap::VisualMap(std::shared_ptr<beam_calibration::CameraModel> cam_model,
                     const std::string &source, const size_t tracked_features,
                     const size_t window_size)
    : cam_model_(cam_model), source_(source),
      tracked_features_(tracked_features), window_size_(window_size) {}

beam::opt<Eigen::Matrix4d> VisualMap::GetCameraPose(const ros::Time &stamp) {
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
    beam::QuaternionAndTranslationToTransformMatrix(orientation, position,
                                                    T_WORLD_BASELINK);

    // transform pose from baselink coord space to camera coord space
    Eigen::Matrix4d T_WORLD_CAMERA =
        T_WORLD_BASELINK * T_cam_baselink_.inverse();
    return T_WORLD_CAMERA;
  } else {
    return {};
  }
}

beam::opt<Eigen::Matrix4d> VisualMap::GetBaselinkPose(const ros::Time &stamp) {
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
    beam::QuaternionAndTranslationToTransformMatrix(orientation, position,
                                                    T_WORLD_BASELINK);
    return T_WORLD_BASELINK;
  } else {
    return {};
  }
}

fuse_variables::Point3DLandmark::SharedPtr
VisualMap::GetLandmark(uint64_t landmark_id) {
  fuse_variables::Point3DLandmark::SharedPtr landmark =
      fuse_variables::Point3DLandmark::make_shared();
  auto landmark_uuid = fuse_core::uuid::generate(landmark->type(), landmark_id);
  if (graph_) {
    try {
      *landmark = dynamic_cast<const fuse_variables::Point3DLandmark &>(
          graph_->getVariable(landmark_uuid));
      return landmark;
    } catch (const std::out_of_range &oor) {
      if (landmark_positions_.find(landmark_id) == landmark_positions_.end()) {
        return nullptr;
      } else {
        return landmark_positions_[landmark_id];
      }
    }
  } else if (landmark_positions_.find(landmark_id) !=
             landmark_positions_.end()) {
    return landmark_positions_[landmark_id];
  }

  return nullptr;
}

fuse_variables::Point3DFixedLandmark::SharedPtr
VisualMap::GetFixedLandmark(uint64_t landmark_id) {
  fuse_variables::Point3DFixedLandmark::SharedPtr landmark =
      fuse_variables::Point3DFixedLandmark::make_shared();
  auto landmark_uuid = fuse_core::uuid::generate(landmark->type(), landmark_id);
  if (graph_) {
    try {
      *landmark = dynamic_cast<const fuse_variables::Point3DFixedLandmark &>(
          graph_->getVariable(landmark_uuid));
      return landmark;
    } catch (const std::out_of_range &oor) {
      if (fixed_landmark_positions_.find(landmark_id) ==
          fixed_landmark_positions_.end()) {
        return nullptr;
      } else {
        return fixed_landmark_positions_[landmark_id];
      }
    }
  } else if (fixed_landmark_positions_.find(landmark_id) !=
             fixed_landmark_positions_.end()) {
    return fixed_landmark_positions_[landmark_id];
  }

  return nullptr;
}

fuse_variables::Orientation3DStamped::SharedPtr
VisualMap::GetOrientation(const ros::Time &stamp) {
  fuse_variables::Orientation3DStamped::SharedPtr corr_orientation =
      fuse_variables::Orientation3DStamped::make_shared();
  auto corr_orientation_uuid = fuse_core::uuid::generate(
      corr_orientation->type(), stamp, fuse_core::uuid::NIL);
  if (graph_) {
    try {
      *corr_orientation =
          dynamic_cast<const fuse_variables::Orientation3DStamped &>(
              graph_->getVariable(corr_orientation_uuid));
      return corr_orientation;
    } catch (const std::out_of_range &oor) {
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

fuse_variables::Position3DStamped::SharedPtr
VisualMap::GetPosition(const ros::Time &stamp) {
  fuse_variables::Position3DStamped::SharedPtr corr_position =
      fuse_variables::Position3DStamped::make_shared();
  auto corr_position_uuid = fuse_core::uuid::generate(
      corr_position->type(), stamp, fuse_core::uuid::NIL);
  if (graph_) {
    try {
      *corr_position = dynamic_cast<const fuse_variables::Position3DStamped &>(
          graph_->getVariable(corr_position_uuid));
      return corr_position;
    } catch (const std::out_of_range &oor) {
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

void VisualMap::AddCameraPose(const Eigen::Matrix4d &T_WORLD_CAMERA,
                              const ros::Time &stamp,
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

void VisualMap::AddBaselinkPose(const Eigen::Matrix4d &T_WORLD_BASELINK,
                                const ros::Time &stamp,
                                fuse_core::Transaction::SharedPtr transaction) {
  Eigen::Quaterniond q;
  Eigen::Vector3d p;
  beam::TransformMatrixToQuaternionAndTranslation(T_WORLD_BASELINK, q, p);

  // add position variable
  AddPosition(p, stamp, transaction);

  // add orientation variable
  AddOrientation(q, stamp, transaction);
}

void VisualMap::AddOrientation(const Eigen::Quaterniond &q_WORLD_BASELINK,
                               const ros::Time &stamp,
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

void VisualMap::AddPosition(const Eigen::Vector3d &p_WORLD_BASELINK,
                            const ros::Time &stamp,
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

void VisualMap::AddLandmark(const Eigen::Vector3d &position, uint64_t id,
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

void VisualMap::AddFixedLandmark(
    const Eigen::Vector3d &position, uint64_t id,
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

void VisualMap::AddOrientation(
    fuse_variables::Orientation3DStamped::SharedPtr orientation,
    fuse_core::Transaction::SharedPtr transaction) {
  // clear local map
  if (orientations_.size() > window_size_) {
    orientations_.erase(orientations_.begin());
  }

  // add to transaction
  transaction->addVariable(orientation);
  orientations_[orientation->stamp().toNSec()] = orientation;
}

void VisualMap::AddPosition(
    fuse_variables::Position3DStamped::SharedPtr position,
    fuse_core::Transaction::SharedPtr transaction) {
  // clear local map
  if (positions_.size() > window_size_) {
    positions_.erase(positions_.begin());
  }

  // add to transaction
  transaction->addVariable(position);
  positions_[position->stamp().toNSec()] = position;
}

void VisualMap::AddLandmark(fuse_variables::Point3DLandmark::SharedPtr landmark,
                            fuse_core::Transaction::SharedPtr transaction) {
  // clear local map
  if (landmark_positions_.size() > window_size_ * tracked_features_) {
    landmark_positions_.erase(landmark_positions_.begin());
  }

  // add to transaction
  transaction->addVariable(landmark);
  landmark_positions_[landmark->id()] = landmark;
}

void VisualMap::AddFixedLandmark(
    fuse_variables::Point3DFixedLandmark::SharedPtr landmark,
    fuse_core::Transaction::SharedPtr transaction) { // clear local map
  if (fixed_landmark_positions_.size() > window_size_ * tracked_features_) {
    fixed_landmark_positions_.erase(fixed_landmark_positions_.begin());
  }

  // add to transaction
  transaction->addVariable(landmark);
  fixed_landmark_positions_[landmark->id()] = landmark;
}

void VisualMap::AddConstraint(const ros::Time &stamp, uint64_t lm_id,
                              const Eigen::Vector2d &pixel,
                              fuse_core::Transaction::SharedPtr transaction) {
  if (!extrinsics_.GetT_CAMERA_BASELINK(T_cam_baselink_)) {
    ROS_ERROR("Unable to get baselink to camera transform.");
    return;
  }

  // get landmark (fixed or not fixed)
  fuse_variables::Point3DLandmark::SharedPtr lm = GetLandmark(lm_id);
  fuse_variables::Point3DFixedLandmark::SharedPtr lm_fixed =
      GetFixedLandmark(lm_id);

  // get robot pose
  fuse_variables::Position3DStamped::SharedPtr position = GetPosition(stamp);
  fuse_variables::Orientation3DStamped::SharedPtr orientation =
      GetOrientation(stamp);
  if (position && orientation) {
    try {
      if (lm) {
        // add normal visual constraint
        fuse_constraints::VisualConstraint::SharedPtr vis_constraint =
            fuse_constraints::VisualConstraint::make_shared(
                source_, *orientation, *position, *lm, pixel, T_cam_baselink_,
                cam_model_);
        transaction->addConstraint(vis_constraint);
      } else if (lm_fixed) {
        // add fixed visual constraint
        fuse_constraints::VisualConstraintFixed::SharedPtr vis_constraint =
            fuse_constraints::VisualConstraintFixed::make_shared(
                source_, *orientation, *position, *lm_fixed, pixel,
                T_cam_baselink_, cam_model_);
        transaction->addConstraint(vis_constraint);
      }
    } catch (const std::logic_error &le) {
    }
  }
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

fuse_core::UUID VisualMap::GetPositionUUID(const ros::Time &stamp) {
  fuse_variables::Position3DStamped::SharedPtr corr_position =
      fuse_variables::Position3DStamped::make_shared();
  auto corr_position_uuid = fuse_core::uuid::generate(
      corr_position->type(), stamp, fuse_core::uuid::NIL);
  return corr_position_uuid;
}

fuse_core::UUID VisualMap::GetOrientationUUID(const ros::Time &stamp) {
  fuse_variables::Orientation3DStamped::SharedPtr corr_orientation =
      fuse_variables::Orientation3DStamped::make_shared();
  auto corr_orientation_uuid = fuse_core::uuid::generate(
      corr_orientation->type(), stamp, fuse_core::uuid::NIL);
  return corr_orientation_uuid;
}

bool VisualMap::PoseExists(const ros::Time &stamp) {
  if (graph_->variableExists(GetOrientationUUID(stamp)) &&
      graph_->variableExists(GetPositionUUID(stamp))) {
    return true;
  }
  return false;
}

bool VisualMap::LandmarkExists(uint64_t landmark_id) {
  if (graph_->variableExists(GetLandmarkUUID(landmark_id))) {
    return true;
  }
  return false;
}

bool VisualMap::FixedLandmarkExists(uint64_t landmark_id) {
  if (graph_->variableExists(GetFixedLandmarkUUID(landmark_id))) {
    return true;
  }
  return false;
}

bool VisualMap::UpdateLandmarks() {
  // update local copy of landmarks
  for (auto &lm : landmark_positions_) {
    fuse_variables::Point3DLandmark::SharedPtr landmark =
        fuse_variables::Point3DLandmark::make_shared();
    auto landmark_uuid = fuse_core::uuid::generate(landmark->type(), lm.first);
    try {
      *landmark = dynamic_cast<const fuse_variables::Point3DLandmark &>(
          graph_->getVariable(landmark_uuid));
      landmark_positions_[lm.first] = landmark;
    } catch (const std::out_of_range &oor) {
    }
  }
}

bool VisualMap::UpdateFixedLandmarks() {
  // update fixed landmarks
  for (auto &flm : fixed_landmark_positions_) {
    fuse_variables::Point3DFixedLandmark::SharedPtr fixed_landmark =
        fuse_variables::Point3DFixedLandmark::make_shared();
    auto fixed_landmark_uuid =
        fuse_core::uuid::generate(fixed_landmark->type(), flm.first);
    try {
      *fixed_landmark =
          dynamic_cast<const fuse_variables::Point3DFixedLandmark &>(
              graph_->getVariable(fixed_landmark_uuid));
      fixed_landmark_positions_[flm.first] = fixed_landmark;
    } catch (const std::out_of_range &oor) {
    }
  }
}

bool VisualMap::UpdatePositions() {
  // update positions
  for (auto &p : positions_) {
    ros::Time stamp = ros::Time::now();
    stamp.fromNSec(p.first);
    fuse_variables::Position3DStamped::SharedPtr position =
        fuse_variables::Position3DStamped::make_shared();
    auto position_uuid = fuse_core::uuid::generate(position->type(), stamp,
                                                   fuse_core::uuid::NIL);
    try {
      *position = dynamic_cast<const fuse_variables::Position3DStamped &>(
          graph_->getVariable(position_uuid));
      positions_[p.first] = position;
    } catch (const std::out_of_range &oor) {
    }
  }
}

bool VisualMap::UpdateOrientations() {
  // update orientations
  for (auto &o : orientations_) {
    ros::Time stamp = ros::Time::now();
    stamp.fromNSec(o.first);
    fuse_variables::Orientation3DStamped::SharedPtr orientation =
        fuse_variables::Orientation3DStamped::make_shared();
    auto orientation_uuid = fuse_core::uuid::generate(
        orientation->type(), stamp, fuse_core::uuid::NIL);
    try {
      *orientation = dynamic_cast<const fuse_variables::Orientation3DStamped &>(
          graph_->getVariable(orientation_uuid));
      orientations_[o.first] = orientation;
    } catch (const std::out_of_range &oor) {
    }
  }
}

void VisualMap::UpdateGraph(fuse_core::Graph::SharedPtr graph_msg) {
  graph_ = std::move(graph_msg);
  // update local copies of variables with the new graph
  UpdateLandmarks();
  UpdateFixedLandmarks();
  UpdatePositions();
  UpdateOrientations();
}

} // namespace vision
} // namespace bs_models