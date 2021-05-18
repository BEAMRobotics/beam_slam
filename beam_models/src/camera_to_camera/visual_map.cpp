#include <beam_constraints/camera_to_camera/visual_constraint.h>
#include <beam_models/camera_to_camera/visual_map.h>

namespace beam_models { namespace camera_to_camera {

VisualMap::VisualMap(std::string source,
                     std::shared_ptr<beam_calibration::CameraModel> cam_model)
    : source_(source), cam_model_(cam_model) {}

fuse_variables::Orientation3DStamped::SharedPtr
    VisualMap::getOrientation(const ros::Time& stamp) {
  std::string orientation_3d_stamped_type =
      "fuse_variables::Orientation3DStamped";
  fuse_variables::Orientation3DStamped::SharedPtr corr_orientation =
      fuse_variables::Orientation3DStamped::make_shared();
  // first check the graph for the variable if its initialized
  if (graph_initialized) {
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
    VisualMap::getPosition(const ros::Time& stamp) {
  std::string position_3d_stamped_type = "fuse_variables::Position3DStamped";
  fuse_variables::Position3DStamped::SharedPtr corr_position =
      fuse_variables::Position3DStamped::make_shared();
  // first check the graph for the variable if its initialized
  if (graph_initialized) {
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

fuse_variables::Position3D::SharedPtr
    VisualMap::getLandmark(uint64_t landmark_id) {
  std::string position_3d_type = "fuse_variables::Position3D";
  fuse_variables::Position3D::SharedPtr landmark =
      fuse_variables::Position3D::make_shared();
  // first check the graph for the variable if its initialized
  if (graph_initialized) {
    auto landmark_uuid = fuse_core::uuid::generate(
        position_3d_type, std::to_string(landmark_id).c_str());
    try {
      *landmark = dynamic_cast<const fuse_variables::Position3D&>(
          graph_->getVariable(landmark_uuid));
      landmark_positions_.erase(landmark_id);
    } catch (const std::out_of_range& oor) {
      if (landmark_positions_.find(landmark_id) == landmark_positions_.end()) {
        return nullptr;
      } else {
        return landmark_positions_[landmark_id];
      }
    }
  }
  // if its not initalized check local maps
  if (landmark_positions_.find(landmark_id) != landmark_positions_.end()) {
    landmark_positions_[landmark_id];
  }
  return nullptr;
}

void VisualMap::addOrientation(
    const Eigen::Quaterniond& q, const ros::Time& cur_time,
    std::shared_ptr<fuse_core::Transaction> transaction) {
  fuse_variables::Orientation3DStamped::SharedPtr orientation =
      fuse_variables::Orientation3DStamped::make_shared(cur_time);
  orientation->w() = q.w();
  orientation->x() = q.x();
  orientation->y() = q.y();
  orientation->z() = q.z();
  transaction->addVariable(orientation);
  orientations_[cur_time.toNSec()] = orientation;
}

void VisualMap::addPosition(
    const Eigen::Vector3d& p, const ros::Time& cur_time,
    std::shared_ptr<fuse_core::Transaction> transaction) {
  fuse_variables::Position3DStamped::SharedPtr position =
      fuse_variables::Position3DStamped::make_shared(cur_time);
  position->x() = p[0];
  position->y() = p[1];
  position->z() = p[2];
  transaction->addVariable(position);
  positions_[cur_time.toNSec()] = position;
}

void VisualMap::addLandmark(
    const Eigen::Vector3d& p, uint64_t id,
    std::shared_ptr<fuse_core::Transaction> transaction) {
  fuse_variables::Position3D::SharedPtr landmark =
      fuse_variables::Position3D::make_shared(std::to_string(id).c_str());
  landmark->x() = p[0];
  landmark->y() = p[1];
  landmark->z() = p[2];
  transaction->addVariable(landmark);
  landmark_positions_[id] = landmark;
}

void VisualMap::addConstraint(
    const ros::Time& img_time, uint64_t lm_id, const Eigen::Vector2d& pixel,
    std::shared_ptr<fuse_core::Transaction> transaction) {
  fuse_variables::Position3D::SharedPtr lm = this->getLandmark(lm_id);
  fuse_variables::Position3DStamped::SharedPtr cam_pos =
      this->getPosition(img_time);
  fuse_variables::Orientation3DStamped::SharedPtr cam_or =
      this->getOrientation(img_time);
  fuse_constraints::VisualConstraint::SharedPtr vis_constraint =
      fuse_constraints::VisualConstraint::make_shared(
          source_, *cam_or, *cam_pos, *lm, pixel, cam_model_);
  transaction->addConstraint(vis_constraint);
}

void VisualMap::updateGraph(fuse_core::Graph::ConstSharedPtr graph_msg) {
  this->graph_ = std::move(graph_msg);
  if (!graph_initialized) graph_initialized = true;
}

}} // namespace beam_models::camera_to_camera