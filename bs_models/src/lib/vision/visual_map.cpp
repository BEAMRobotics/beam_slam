#include <bs_models/vision/visual_map.h>

#include <beam_utils/math.h>
#include <beam_utils/se3.h>

#include <bs_constraints/helpers.h>

#include <bs_common/conversions.h>
#include <bs_common/graph_access.h>
#include <bs_constraints/visual/euclidean_reprojection_constraint.h>
#include <bs_constraints/visual/inversedepth_reprojection_constraint.h>
#include <bs_constraints/visual/inversedepth_reprojection_constraint_unary.h>
#include <fuse_constraints/absolute_pose_3d_stamped_constraint.h>
#include <fuse_constraints/relative_pose_3d_stamped_constraint.h>

namespace bs_models { namespace vision {

VisualMap::VisualMap(const std::string& source,
                     std::shared_ptr<beam_calibration::CameraModel> cam_model,
                     fuse_core::Loss::SharedPtr loss_function,
                     const double reprojection_information_weight)
    : source_(source),
      cam_model_(cam_model),
      loss_function_(loss_function),
      reprojection_information_weight_(reprojection_information_weight) {
  cam_model_->InitUndistortMap();
  camera_intrinsic_matrix_ =
      cam_model->GetRectifiedModel()->GetIntrinsicMatrix();
  if (!extrinsics_.GetT_CAMERA_BASELINK(T_cam_baselink_)) {
    ROS_ERROR("Unable to get baselink to camera transform.");
    throw std::runtime_error("Unable to get baselink to camera transform.");
  }
}

beam::opt<Eigen::Matrix4d> VisualMap::GetCameraPose(const ros::Time& stamp) {
  const auto& baselink_pose = GetBaselinkPose(stamp);
  if (baselink_pose.has_value()) {
    Eigen::Matrix4d T_WORLD_BASELINK = baselink_pose.value();
    // transform pose from baselink coord space to camera coord space
    Eigen::Matrix4d T_WORLD_CAMERA =
        T_WORLD_BASELINK * beam::InvertTransform(T_cam_baselink_);
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

bs_variables::InverseDepthLandmark::SharedPtr
    VisualMap::GetInverseDepthLandmark(uint64_t landmark_id) {
  bs_variables::InverseDepthLandmark::SharedPtr landmark =
      bs_variables::InverseDepthLandmark::make_shared();
  auto landmark_uuid = fuse_core::uuid::generate(landmark->type(), landmark_id);
  if (graph_) {
    try {
      *landmark = dynamic_cast<const bs_variables::InverseDepthLandmark&>(
          graph_->getVariable(landmark_uuid));
      return landmark;
    } catch (const std::out_of_range& oor) {
      if (inversedepth_landmark_positions_.find(landmark_id) ==
          inversedepth_landmark_positions_.end()) {
        return nullptr;
      } else {
        return inversedepth_landmark_positions_[landmark_id];
      }
    }
  } else if (inversedepth_landmark_positions_.find(landmark_id) !=
             inversedepth_landmark_positions_.end()) {
    return inversedepth_landmark_positions_[landmark_id];
  }

  return nullptr;
}

bs_variables::Point3DLandmark::SharedPtr
    VisualMap::GetLandmark(uint64_t landmark_id) {
  bs_variables::Point3DLandmark::SharedPtr landmark =
      bs_variables::Point3DLandmark::make_shared();
  auto landmark_uuid = fuse_core::uuid::generate(landmark->type(), landmark_id);
  if (graph_) {
    try {
      *landmark = dynamic_cast<const bs_variables::Point3DLandmark&>(
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
  bs_variables::Point3DLandmark::SharedPtr lm = GetLandmark(lm_id);

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
              camera_intrinsic_matrix_, measurement,
              reprojection_information_weight_);
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

void VisualMap::AddLandmark(const Eigen::Vector3d& position,
                            const Eigen::Vector3d& viewing_angle,
                            const uint64_t word_id, const uint64_t id,
                            fuse_core::Transaction::SharedPtr transaction) {
  // construct landmark variable
  bs_variables::Point3DLandmark::SharedPtr landmark =
      bs_variables::Point3DLandmark::make_shared(id, viewing_angle, word_id);
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

void VisualMap::AddLandmark(bs_variables::Point3DLandmark::SharedPtr landmark,
                            fuse_core::Transaction::SharedPtr transaction) {
  // add to transaction
  transaction->addVariable(landmark);
  landmark_positions_[landmark->id()] = landmark;
}

void VisualMap::AddInverseDepthLandmark(
    const Eigen::Vector3d& bearing, const double inverse_depth,
    const uint64_t id, const ros::Time& anchor_time,
    fuse_core::Transaction::SharedPtr transaction) {
  // construct landmark variable
  bs_variables::InverseDepthLandmark::SharedPtr landmark =
      bs_variables::InverseDepthLandmark::make_shared(id, bearing, anchor_time);
  landmark->inverse_depth() = inverse_depth;

  // add fuse landmark variable
  AddInverseDepthLandmark(landmark, transaction);
}

void VisualMap::AddInverseDepthLandmark(
    bs_variables::InverseDepthLandmark::SharedPtr landmark,
    fuse_core::Transaction::SharedPtr transaction) {
  // add to transaction
  transaction->addVariable(landmark);
  inversedepth_landmark_positions_[landmark->id()] = landmark;
}

bool VisualMap::AddInverseDepthVisualConstraint(
    const ros::Time& measurement_stamp, uint64_t lm_id,
    const Eigen::Vector2d& pixel,
    fuse_core::Transaction::SharedPtr transaction) {
  // get landmark
  bs_variables::InverseDepthLandmark::SharedPtr lm =
      GetInverseDepthLandmark(lm_id);

  // get measurement robot pose
  fuse_variables::Position3DStamped::SharedPtr position_m =
      GetPosition(measurement_stamp);
  fuse_variables::Orientation3DStamped::SharedPtr orientation_m =
      GetOrientation(measurement_stamp);

  // get anchor robot pose
  fuse_variables::Position3DStamped::SharedPtr position_a =
      GetPosition(lm->anchorStamp());
  fuse_variables::Orientation3DStamped::SharedPtr orientation_a =
      GetOrientation(lm->anchorStamp());

  // rectify pixel
  Eigen::Vector2i rectified_pixel;
  if (!cam_model_->UndistortPixel(pixel.cast<int>(), rectified_pixel)) {
    return false;
  }
  Eigen::Vector2d measurement = rectified_pixel.cast<double>();

  if (!position_a || !orientation_a || !position_m || !orientation_m) {
    return false;
  }
  try {
    if (lm) {
      if (lm->anchorStamp() == measurement_stamp) {
        auto vis_constraint = std::make_shared<
            bs_constraints::InverseDepthReprojectionConstraintUnary>(
            source_, *orientation_a, *position_a, *lm, T_cam_baselink_,
            camera_intrinsic_matrix_, measurement,
            reprojection_information_weight_);
        vis_constraint->loss(loss_function_);
        transaction->addConstraint(vis_constraint);
      } else {
        auto vis_constraint = std::make_shared<
            bs_constraints::InverseDepthReprojectionConstraint>(
            source_, *orientation_a, *position_a, *orientation_m, *position_m,
            *lm, T_cam_baselink_, camera_intrinsic_matrix_, measurement,
            reprojection_information_weight_);
        vis_constraint->loss(loss_function_);
        transaction->addConstraint(vis_constraint);
      }
      return true;
    }
  } catch (const std::logic_error& le) {}

  return false;
}

fuse_core::UUID VisualMap::GetLandmarkUUID(uint64_t landmark_id) {
  bs_variables::Point3DLandmark::SharedPtr landmark =
      bs_variables::Point3DLandmark::make_shared();
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

void VisualMap::AddPosePrior(const ros::Time& stamp,
                             const Eigen::Matrix<double, 6, 6>& covariance,
                             fuse_core::Transaction::SharedPtr transaction) {
  const auto position = GetPosition(stamp);
  const auto orientation = GetOrientation(stamp);
  if (position && orientation) {
    fuse_core::Vector7d mean;
    mean << position->x(), position->y(), position->z(), orientation->w(),
        orientation->x(), orientation->y(), orientation->z();

    auto prior =
        std::make_shared<fuse_constraints::AbsolutePose3DStampedConstraint>(
            source_, *position, *orientation, mean, covariance);
    transaction->addConstraint(prior);
  }
}

void VisualMap::AddRelativePoseConstraint(
    const ros::Time& stamp1, const ros::Time& stamp2,
    const fuse_core::Vector7d& delta,
    const Eigen::Matrix<double, 6, 6>& covariance,
    fuse_core::Transaction::SharedPtr transaction) {
  const auto position1 = GetPosition(stamp1);
  const auto orientation1 = GetOrientation(stamp1);
  const auto position2 = GetPosition(stamp2);
  const auto orientation2 = GetOrientation(stamp2);
  if (position1 && orientation1 && position2 && orientation2) {
    auto constraint =
        std::make_shared<fuse_constraints::RelativePose3DStampedConstraint>(
            source_, *position1, *orientation1, *position2, *orientation2,
            delta, covariance);
    transaction->addConstraint(constraint);
  }
}

void VisualMap::UpdateGraph(const fuse_core::Graph& graph_msg) {
  graph_ = std::move(graph_msg.clone());

  // remove local copies of poses that are in the new graph
  const auto graph_timestamps = bs_common::CurrentTimestamps(graph_msg);
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
  const auto graph_lm_ids = bs_common::CurrentLandmarkIDs(graph_msg);
  std::vector<uint64_t> lms_to_remove;
  for (const auto [id, position] : landmark_positions_) {
    if (graph_lm_ids.find(id) != graph_lm_ids.end()) {
      lms_to_remove.push_back(id);
    }
  }
  for (const auto [id, position] : inversedepth_landmark_positions_) {
    if (graph_lm_ids.find(id) != graph_lm_ids.end()) {
      lms_to_remove.push_back(id);
    }
  }

  for (const auto& id : lms_to_remove) { landmark_positions_.erase(id); }
  for (const auto& id : lms_to_remove) {
    inversedepth_landmark_positions_.erase(id);
  }
}

void VisualMap::Clear() {
  orientations_.clear();
  positions_.clear();
  landmark_positions_.clear();

  inversedepth_landmark_positions_.clear();
  if (graph_) { graph_ = nullptr; }
}

std::set<ros::Time> VisualMap::CurrentTimestamps() {
  auto graph_timestamps = bs_common::CurrentTimestamps(*graph_);
  for (const auto& [t, pos] : positions_) {
    graph_timestamps.insert(beam::NSecToRos(t));
  }
  return graph_timestamps;
}

std::map<uint64_t, Eigen::Vector3d> VisualMap::GetLandmarks() {
  std::map<uint64_t, Eigen::Vector3d> landmarks;
  std::set<uint64_t> graph_lms = bs_common::CurrentLandmarkIDs(*graph_);
  for (const auto& id : graph_lms) {
    landmarks[id] = bs_common::GetLandmark(*graph_, id)->point();
  }
  for (const auto& [id, landmark] : landmark_positions_) {
    landmarks[id] = landmark->point();
  }
  return landmarks;
}

std::set<uint64_t> VisualMap::GetLandmarkIDs() {
  std::set<uint64_t> graph_lms = bs_common::CurrentLandmarkIDs(*graph_);
  for (const auto& [id, landmark] : landmark_positions_) {
    graph_lms.insert(id);
  }
  return graph_lms;
}

}} // namespace bs_models::vision