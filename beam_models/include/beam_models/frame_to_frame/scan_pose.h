#pragma once

#include <beam_utils/pointclouds.h>

#include <fuse_core/graph.h>
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>

namespace beam_models { namespace frame_to_frame {

class ScanPose {
public:
  ScanPose() = default;

  ScanPose(const ros::Time& time, const Eigen::Matrix4d& T_WORLD_CLOUD,
           PointCloudPtr pc)
      : stamp_(time), pointcloud_(pc), T_WORLD_CLOUD_(T_WORLD_CLOUD) {
    position_uuid_ = fuse_core::uuid::generate(
        "fuse_variables::Position3DStamped", time, fuse_core::uuid::NIL);
    orientation_uuid_ = fuse_core::uuid::generate(
        "fuse_variables::Orientation3DStamped", time, fuse_core::uuid::NIL);
  }

  void Update(const fuse_core::Graph::ConstSharedPtr& graph_msg) {
    *position_ = dynamic_cast<const fuse_variables::Position3DStamped&>(
        graph_msg->getVariable(position_uuid_));

    *orientation_ = dynamic_cast<const fuse_variables::Orientation3DStamped&>(
        graph_msg->getVariable(orientation_uuid_));

    Eigen::Quaterniond q(orientation_->w(), orientation_->x(),
                         orientation_->y(), orientation_->z());
    T_WORLD_CLOUD_.block(0, 3, 3, 1) =
        Eigen::Vector3d{position_->x(), position_->y(), position_->z()};
    T_WORLD_CLOUD_.block(0, 0, 3, 3) = q.toRotationMatrix();
  }

  bool Near(const ros::Time& time, const double tolerance) const {
    return (std::abs(stamp_.toSec() - time.toSec()) <= tolerance);
  }

  bool operator<(const ScanPose& rhs) const { return (stamp_ < rhs.stamp_); }

  fuse_variables::Position3DStamped::SharedPtr Position() const {
    return position_;
  }

  fuse_variables::Orientation3DStamped::SharedPtr Orientation() const {
    return orientation_;
  }

  Eigen::Matrix4d T_WORLD_CLOUD() const { return T_WORLD_CLOUD_; }

  PointCloudPtr Cloud() const { return pointcloud_; }

  ros::Time Stamp() const { return stamp_; }

  using Ptr = std::shared_ptr<ScanPose>;

protected:
  fuse_core::UUID position_uuid_ = fuse_core::uuid::NIL;
  fuse_core::UUID orientation_uuid_ = fuse_core::uuid::NIL;
  ros::Time stamp_;
  Eigen::Matrix4d T_WORLD_CLOUD_;
  fuse_variables::Position3DStamped::SharedPtr position_ =
      fuse_variables::Position3DStamped::make_shared();
  fuse_variables::Orientation3DStamped::SharedPtr orientation_ =
      fuse_variables::Orientation3DStamped::make_shared();
  PointCloudPtr pointcloud_;
};

}} // namespace beam_models::frame_to_frame
