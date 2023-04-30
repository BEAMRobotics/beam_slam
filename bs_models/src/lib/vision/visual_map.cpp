#include <bs_common/utils.h>
#include <bs_models/vision/visual_map.h>

#include <beam_utils/math.h>
#include <beam_utils/se3.h>

#include <bs_constraints/visual/euclidean_reprojection_constraint.h>

namespace bs_models { namespace vision {

VisualMap::VisualMap(std::shared_ptr<beam_calibration::CameraModel> cam_model,
                     fuse_core::Loss::SharedPtr loss_function,
                     const Eigen::Matrix2d& covariance)
    : cam_model_(cam_model),
      loss_function_(loss_function),
      covariance_(covariance) {
  cam_model_->InitUndistortMap();
  camera_intrinsic_matrix_ =
      cam_model->GetRectifiedModel()->GetIntrinsicMatrix();
  if (!extrinsics_.GetT_CAMERA_BASELINK(T_cam_baselink_)) {
    ROS_ERROR("Unable to get baselink to camera transform.");
    throw std::runtime_error("Unable to get baselink to camera transform.");
  }
}

beam::opt<Eigen::Matrix4d> VisualMap::GetCameraPose(const ros::Time& stamp) {
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

beam::opt<Eigen::Matrix4d> VisualMap::GetBaselinkPose(const ros::Time& stamp) {
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
      *landmark = dynamic_cast<const fuse_variables::Point3DLandmark&>(
          graph_->getVariable(landmark_uuid));
      return landmark;
    } catch (const std::out_of_range& oor) {
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

fuse_variables::Orientation3DStamped::SharedPtr
    VisualMap::GetOrientation(const ros::Time& stamp) {
  fuse_variables::Orientation3DStamped::SharedPtr corr_orientation =
      fuse_variables::Orientation3DStamped::make_shared();
  auto corr_orientation_uuid = fuse_core::uuid::generate(
      corr_orientation->type(), stamp, fuse_core::uuid::NIL);
  if (graph_) {
    try {
      *corr_orientation =
          dynamic_cast<const fuse_variables::Orientation3DStamped&>(
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

fuse_variables::Position3DStamped::SharedPtr
    VisualMap::GetPosition(const ros::Time& stamp) {
  fuse_variables::Position3DStamped::SharedPtr corr_position =
      fuse_variables::Position3DStamped::make_shared();
  auto corr_position_uuid = fuse_core::uuid::generate(
      corr_position->type(), stamp, fuse_core::uuid::NIL);
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

bool VisualMap::AddVisualConstraint(
    const ros::Time& stamp, uint64_t lm_id, const Eigen::Vector2d& pixel,
    fuse_core::Transaction::SharedPtr transaction) {
  // get landmark
  fuse_variables::Point3DLandmark::SharedPtr lm = GetLandmark(lm_id);

  // get robot pose
  fuse_variables::Position3DStamped::SharedPtr position = GetPosition(stamp);
  fuse_variables::Orientation3DStamped::SharedPtr orientation =
      GetOrientation(stamp);

  // rectify pixel
  Eigen::Vector2i rectified_pixel;
  if (!cam_model_->UndistortPixel(pixel.cast<int>(), rectified_pixel)) {
    return false;
  }
  Eigen::Vector2d measurement = rectified_pixel.cast<double>();

  if (!position || !orientation) { return false; }
  try {
    if (lm) {
      auto vis_constraint =
          std::make_shared<bs_constraints::EuclideanReprojectionConstraint>(
              source_, *orientation, *position, *lm, T_cam_baselink_,
              camera_intrinsic_matrix_, measurement, covariance_);
      vis_constraint->loss(loss_function_);
      transaction->addConstraint(vis_constraint);
      return true;
    }
  } catch (const std::logic_error& le) {}

  return false;
}

void VisualMap::AddCameraPose(const Eigen::Matrix4d& T_WORLD_CAMERA,
                              const ros::Time& stamp,
                              fuse_core::Transaction::SharedPtr transaction) {
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

void VisualMap::AddBaselinkPose(const Eigen::Matrix4d& T_WORLD_BASELINK,
                                const ros::Time& stamp,
                                fuse_core::Transaction::SharedPtr transaction) {
  Eigen::Quaterniond q;
  Eigen::Vector3d p;
  beam::TransformMatrixToQuaternionAndTranslation(T_WORLD_BASELINK, q, p);

  // add position variable
  AddPosition(p, stamp, transaction);

  // add orientation variable
  AddOrientation(q, stamp, transaction);
}

void VisualMap::AddOrientation(const Eigen::Quaterniond& q_WORLD_BASELINK,
                               const ros::Time& stamp,
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

void VisualMap::AddPosition(const Eigen::Vector3d& p_WORLD_BASELINK,
                            const ros::Time& stamp,
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

void VisualMap::AddOrientation(
    fuse_variables::Orientation3DStamped::SharedPtr orientation,
    fuse_core::Transaction::SharedPtr transaction) {
  // add to transaction
  transaction->addVariable(orientation);
  orientations_[orientation->stamp().toNSec()] = orientation;
}

void VisualMap::AddPosition(
    fuse_variables::Position3DStamped::SharedPtr position,
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

fuse_core::UUID VisualMap::GetLandmarkUUID(uint64_t landmark_id) {
  fuse_variables::Point3DLandmark::SharedPtr landmark =
      fuse_variables::Point3DLandmark::make_shared();
  auto landmark_uuid = fuse_core::uuid::generate(landmark->type(), landmark_id);
  return landmark_uuid;
}

fuse_core::UUID VisualMap::GetPositionUUID(const ros::Time& stamp) {
  fuse_variables::Position3DStamped::SharedPtr corr_position =
      fuse_variables::Position3DStamped::make_shared();
  auto corr_position_uuid = fuse_core::uuid::generate(
      corr_position->type(), stamp, fuse_core::uuid::NIL);
  return corr_position_uuid;
}

fuse_core::UUID VisualMap::GetOrientationUUID(const ros::Time& stamp) {
  fuse_variables::Orientation3DStamped::SharedPtr corr_orientation =
      fuse_variables::Orientation3DStamped::make_shared();
  auto corr_orientation_uuid = fuse_core::uuid::generate(
      corr_orientation->type(), stamp, fuse_core::uuid::NIL);
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

void VisualMap::UpdateGraph(fuse_core::Graph::ConstSharedPtr graph_msg) {
  graph_ = graph_msg;

  // remove local copies of poses that are in the new graph
  const auto graph_timestamps = bs_common::CurrentTimestamps(graph_);
  std::vector<uint64_t> times_to_remove;
  for (const auto [t_nsec, position] : positions_) {
    if (graph_timestamps.find(beam::NSecToRos(t_nsec)) !=
        graph_timestamps.end()) {
      times_to_remove.push_back(t_nsec);
    }
  }
  for (const auto& t_nsec : times_to_remove) {
    orientations_.erase(t_nsec);
    positions_.erase(t_nsec);
  }

  // remove local copies of landmarks that are in the new graph
  const auto graph_lm_ids = bs_common::CurrentLandmarkIDs(graph_);
  std::vector<uint64_t> lms_to_remove;
  for (const auto [id, position] : landmark_positions_) {
    if (graph_lm_ids.find(id) != graph_lm_ids.end()) {
      lms_to_remove.push_back(id);
    }
  }
  for (const auto& id : lms_to_remove) { landmark_positions_.erase(id); }
}

void VisualMap::Clear() {
  orientations_.clear();
  positions_.clear();
  landmark_positions_.clear();
  if (graph_) { graph_ = nullptr; }
}

std::set<ros::Time> VisualMap::CurrentTimestamps() {
  auto graph_timestamps = bs_common::CurrentTimestamps(graph_);
  for (const auto& [t, pos] : positions_) {
    graph_timestamps.insert(beam::NSecToRos(t));
  }
  return graph_timestamps;
}

}} // namespace bs_models::vision