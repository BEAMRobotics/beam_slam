#pragma once

#include <beam_utils/math.h>
#include <beam_utils/pointclouds.h>

#include <fuse_core/graph.h>
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

namespace beam_models { namespace frame_to_frame {

class ScanPose {
public:
  ScanPose() = default;

  ScanPose(const ros::Time& time, const Eigen::Matrix4d& T_WORLD_CLOUD,
           const PointCloud& pc)
      : stamp_(time), pointcloud_(pc), T_WORLD_CLOUD_initial_(T_WORLD_CLOUD) {
    // create fuse variables
    position_ = fuse_variables::Position3DStamped(time, fuse_core::uuid::NIL);
    orientation_ =
        fuse_variables::Orientation3DStamped(time, fuse_core::uuid::NIL);

    // add transform
    position_.x() = T_WORLD_CLOUD(0, 3);
    position_.y() = T_WORLD_CLOUD(1, 3);
    position_.z() = T_WORLD_CLOUD(2, 3);
    Eigen::Matrix3d R = T_WORLD_CLOUD.block(0, 0, 3, 3);
    Eigen::Quaterniond q(R);
    orientation_.x() = q.x();
    orientation_.y() = q.y();
    orientation_.z() = q.z();
    orientation_.w() = q.w();
  }

  bool Update(const fuse_core::Graph::ConstSharedPtr& graph_msg) {
    if (graph_msg->variableExists(position_.uuid()) &&
        graph_msg->variableExists(orientation_.uuid())) {
      position_ = dynamic_cast<const fuse_variables::Position3DStamped&>(
          graph_msg->getVariable(position_.uuid()));

      orientation_ = dynamic_cast<const fuse_variables::Orientation3DStamped&>(
          graph_msg->getVariable(orientation_.uuid()));
      updates_++;
      return true;
    }
    return false;
  }

  bool Near(const ros::Time& time, const double tolerance) const {
    return (std::abs(stamp_.toSec() - time.toSec()) <= tolerance);
  }

  int Updates() const { return updates_; }

  bool operator<(const ScanPose& rhs) const { return (stamp_ < rhs.stamp_); }

  fuse_variables::Position3DStamped Position() const { return position_; }

  fuse_variables::Orientation3DStamped Orientation() const {
    return orientation_;
  }

  Eigen::Matrix4d T_WORLD_CLOUD() const {
    Eigen::Matrix4d T_WORLD_CLOUD{Eigen::Matrix4d::Identity()};
    Eigen::Quaterniond q(orientation_.w(), orientation_.x(), orientation_.y(),
                         orientation_.z());
    T_WORLD_CLOUD.block(0, 3, 3, 1) =
        Eigen::Vector3d{position_.x(), position_.y(), position_.z()};
    T_WORLD_CLOUD.block(0, 0, 3, 3) = q.toRotationMatrix();
    return T_WORLD_CLOUD;
  }

  PointCloud Cloud() const { return pointcloud_; }

  ros::Time Stamp() const { return stamp_; }

  void Print(std::ostream& stream = std::cout) const {
    stream << "  Stamp: " << stamp_ << "\n"
           << "  Cloud size: " << pointcloud_.size() << "\n"
           << "  Number of Updates: " << updates_ << "\n"
           << "  Position:\n"
           << "  - x: " << position_.x() << "\n"
           << "  - y: " << position_.y() << "\n"
           << "  - z: " << position_.z() << "\n"
           << "  Orientation:\n"
           << "  - x: " << orientation_.x() << "\n"
           << "  - y: " << orientation_.y() << "\n"
           << "  - z: " << orientation_.z() << "\n"
           << "  - w: " << orientation_.w() << "\n";
  }

  void Save(const std::string& save_path, bool to_world_frame = true,
            bool add_frame = true) {
    std::string file_name_prefix = save_path + std::to_string(stamp_.toSec());
    if (!to_world_frame) {
      pcl::io::savePCDFileASCII(file_name_prefix + ".pcd", pointcloud_);
      return;
    }

    PointCloudPtr cloud_initial = std::make_shared<PointCloud>();
    PointCloudPtr cloud_final = std::make_shared<PointCloud>();

    pcl::transformPointCloud(pointcloud_, *cloud_initial,
                             T_WORLD_CLOUD_initial_);
    pcl::transformPointCloud(pointcloud_, *cloud_final, this->T_WORLD_CLOUD());

    PointCloudColPtr cloud_initial_col =
        beam::ColorPointCloud(cloud_initial, 255, 0, 0);
    PointCloudColPtr cloud_final_col =
        beam::ColorPointCloud(cloud_final, 0, 255, 0);

    if (add_frame) {
      PointCloudColPtr cloud_initial_col_ =
          beam::AddFrameToCloud(cloud_initial_col, T_WORLD_CLOUD_initial_);
      PointCloudColPtr cloud_final_col_ =
          beam::AddFrameToCloud(cloud_final_col, this->T_WORLD_CLOUD());
      pcl::io::savePCDFileASCII(file_name_prefix + "_initial.pcd",
                                *cloud_initial_col_);
      pcl::io::savePCDFileASCII(file_name_prefix + "_final.pcd",
                                *cloud_final_col_);
    } else {
      pcl::io::savePCDFileASCII(file_name_prefix + "_initial.pcd",
                                *cloud_initial_col);
      pcl::io::savePCDFileASCII(file_name_prefix + "_final.pcd",
                                *cloud_final_col);
    }

    ROS_DEBUG("Saved cloud with stamp: %.5f", stamp_.toSec());
  }

  const Eigen::Matrix4d T_WORLD_CLOUD_INIT() const { return T_WORLD_CLOUD_initial_; }

protected:
  ros::Time stamp_;
  int updates_{0};
  fuse_variables::Position3DStamped position_;
  fuse_variables::Orientation3DStamped orientation_;
  PointCloud pointcloud_;
  const Eigen::Matrix4d T_WORLD_CLOUD_initial_;
};

}} // namespace beam_models::frame_to_frame
