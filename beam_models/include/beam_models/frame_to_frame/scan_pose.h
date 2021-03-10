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
      : stamp_(time), pointcloud_(pc) {
    // create fuse variables
    position_ = fuse_variables::Position3DStamped::make_shared(
        time, fuse_core::uuid::NIL);
    orientation_ = fuse_variables::Orientation3DStamped::make_shared(
        time, fuse_core::uuid::NIL);

    // add transform
    position_->x() = T_WORLD_CLOUD(0, 3);
    position_->y() = T_WORLD_CLOUD(1, 3);
    position_->z() = T_WORLD_CLOUD(2, 3);
    Eigen::Matrix3d R = T_WORLD_CLOUD.block(0, 0, 3, 3);
    Eigen::Quaterniond q(R);
    orientation_->x() = q.x();
    orientation_->y() = q.y();
    orientation_->z() = q.z();
    orientation_->w() = q.w();
  }

  void Update(const fuse_core::Graph::ConstSharedPtr& graph_msg) {
    *position_ = dynamic_cast<const fuse_variables::Position3DStamped&>(
        graph_msg->getVariable(position_->uuid()));

    *orientation_ = dynamic_cast<const fuse_variables::Orientation3DStamped&>(
        graph_msg->getVariable(orientation_->uuid()));
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

  Eigen::Matrix4d T_WORLD_CLOUD() const {
    Eigen::Matrix4d T_WORLD_CLOUD;
    Eigen::Quaterniond q(orientation_->w(), orientation_->x(),
                         orientation_->y(), orientation_->z());
    T_WORLD_CLOUD.block(0, 3, 3, 1) =
        Eigen::Vector3d{position_->x(), position_->y(), position_->z()};
    T_WORLD_CLOUD.block(0, 0, 3, 3) = q.toRotationMatrix();
    return T_WORLD_CLOUD;
  }

  PointCloudPtr Cloud() const { return pointcloud_; }

  ros::Time Stamp() const { return stamp_; }

  void Print(std::ostream& stream = std::cout) const {
    stream << "  Stamp: " << stamp_ << "\n"
           << "  Cloud size: " << pointcloud_->size() << "\n"
           << "  Position:\n"
           << "  - x: " << position_->x() << "\n"
           << "  - y: " << position_->y() << "\n"
           << "  - z: " << position_->z() << "\n"
           << "  Orientation:\n"
           << "  - x: " << orientation_->x() << "\n"
           << "  - y: " << orientation_->y() << "\n"
           << "  - z: " << orientation_->z() << "\n"
           << "  - w: " << orientation_->w() << "\n";
  }

  using Ptr = std::shared_ptr<ScanPose>;

protected:
  ros::Time stamp_;
  fuse_variables::Position3DStamped::SharedPtr position_;
  fuse_variables::Orientation3DStamped::SharedPtr orientation_;
  PointCloudPtr pointcloud_;
};

}} // namespace beam_models::frame_to_frame
