#include <bs_common/scan_pose.h>

#include <boost/filesystem.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <beam_utils/math.h>

#include <bs_common/utils.h>

namespace bs_common {

ScanPose::ScanPose(const PointCloud& cloud, const ros::Time& stamp,
                   const Eigen::Matrix4d& T_REFFRAME_BASELINK,
                   const Eigen::Matrix4d& T_BASELINK_LIDAR,
                   const std::shared_ptr<beam_matching::LoamFeatureExtractor>&
                       feature_extractor)
    : pointcloud_(cloud),
      stamp_(stamp),
      T_REFFRAME_BASELINK_initial_(T_REFFRAME_BASELINK),
      T_BASELINK_LIDAR_(T_BASELINK_LIDAR) {
  // create fuse variables
  position_ = fuse_variables::Position3DStamped(stamp, fuse_core::uuid::NIL);
  orientation_ =
      fuse_variables::Orientation3DStamped(stamp, fuse_core::uuid::NIL);

  // add transform
  bs_common::EigenTransformToFusePose(T_REFFRAME_BASELINK, position_,
                                      orientation_);

  if (feature_extractor != nullptr) {
    cloud_type_ = "LOAMPOINTCLOUD";
    loampointcloud_ = feature_extractor->ExtractFeatures(cloud);
  }
}

ScanPose::ScanPose(const ros::Time& stamp,
                   const Eigen::Matrix4d& T_REFFRAME_BASELINK,
                   const Eigen::Matrix4d& T_BASELINK_LIDAR)
    : stamp_(stamp),
      T_REFFRAME_BASELINK_initial_(T_REFFRAME_BASELINK),
      T_BASELINK_LIDAR_(T_BASELINK_LIDAR) {
  // create fuse variables
  position_ = fuse_variables::Position3DStamped(stamp, fuse_core::uuid::NIL);
  orientation_ =
      fuse_variables::Orientation3DStamped(stamp, fuse_core::uuid::NIL);

  // add transform
  bs_common::EigenTransformToFusePose(T_REFFRAME_BASELINK, position_,
                                      orientation_);
}

void ScanPose::AddPointCloud(const PointCloud& cloud, bool override_cloud) {
  if (override_cloud) {
    pointcloud_ = cloud;
  } else {
    pointcloud_ += cloud;
  }
}

void ScanPose::AddPointCloud(const beam_matching::LoamPointCloud& cloud,
                             bool override_cloud) {
  if (override_cloud) {
    loampointcloud_ = cloud;
  } else {
    loampointcloud_.Merge(cloud);
  }
}

void ScanPose::AddPointCloud(const PointCloud& cloud, int type,
                             bool override_cloud) {
  if (type == 0) {
    AddPointCloud(cloud, false);
    return;
  }

  beam_matching::LoamPointCloud new_loam_cloud;
  if (type == 1) {
    new_loam_cloud.AddEdgeFeaturesStrong(cloud);
  } else if (type == 2) {
    new_loam_cloud.AddSurfaceFeaturesStrong(cloud);
  } else if (type == 3) {
    new_loam_cloud.AddEdgeFeaturesWeak(cloud);
  } else if (type == 4) {
    new_loam_cloud.AddSurfaceFeaturesWeak(cloud);
  } else {
    BEAM_ERROR(
        "Invalid pointcloud type, not adding to submap. Input: {}, Options: "
        "0, 1, 2, 3, 4. See LidarMeasurement.msg for more information,",
        type);
    return;
  }
  AddPointCloud(new_loam_cloud, override_cloud);
}

bool ScanPose::UpdatePose(const fuse_core::Graph::ConstSharedPtr& graph_msg) {
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

void ScanPose::UpdatePose(const Eigen::Matrix4d& T_REFFRAME_BASELINK) {
  bs_common::EigenTransformToFusePose(T_REFFRAME_BASELINK, position_,
                                      orientation_);
  updates_++;
}

bool ScanPose::Near(const ros::Time& time, const double tolerance) const {
  return (std::abs(stamp_.toSec() - time.toSec()) <= tolerance);
}

int ScanPose::Updates() const { return updates_; }

bool ScanPose::operator<(const ScanPose& rhs) const {
  return (stamp_ < rhs.stamp_);
}

fuse_variables::Position3DStamped ScanPose::Position() const {
  return position_;
}

fuse_variables::Orientation3DStamped ScanPose::Orientation() const {
  return orientation_;
}

Eigen::Matrix4d ScanPose::T_REFFRAME_BASELINK() const {
  Eigen::Matrix4d T_REFFRAME_BASELINK{Eigen::Matrix4d::Identity()};
  Eigen::Quaterniond q(orientation_.w(), orientation_.x(), orientation_.y(),
                       orientation_.z());
  T_REFFRAME_BASELINK.block(0, 3, 3, 1) =
      Eigen::Vector3d{position_.x(), position_.y(), position_.z()};
  T_REFFRAME_BASELINK.block(0, 0, 3, 3) = q.toRotationMatrix();
  return T_REFFRAME_BASELINK;
}

Eigen::Matrix4d ScanPose::T_REFFRAME_LIDAR() const {
  return T_REFFRAME_BASELINK() * T_BASELINK_LIDAR_;
}

const Eigen::Matrix4d ScanPose::T_REFFRAME_BASELINK_INIT() const {
  return T_REFFRAME_BASELINK_initial_;
}

Eigen::Matrix4d ScanPose::T_REFFRAME_LIDAR_INIT() const {
  return T_REFFRAME_BASELINK_INIT() * T_BASELINK_LIDAR_;
}

Eigen::Matrix4d ScanPose::T_BASELINK_LIDAR() const { return T_BASELINK_LIDAR_; }

Eigen::Matrix4d ScanPose::T_LIDAR_BASELINK() const {
  return beam::InvertTransform(T_BASELINK_LIDAR_);
}

PointCloud ScanPose::Cloud() const { return pointcloud_; }

beam_matching::LoamPointCloud ScanPose::LoamCloud() const {
  return loampointcloud_;
}

ros::Time ScanPose::Stamp() const { return stamp_; }

std::string ScanPose::Type() const { return cloud_type_; }

void ScanPose::SetCloudTypeToLoam() { cloud_type_ = "LOAMPOINTCLOUD"; }

void ScanPose::Print(std::ostream& stream) const {
  stream << "  Stamp: " << stamp_ << "\n"
         << "  Cloud Type: " << cloud_type_ << "\n"
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

void ScanPose::Save(const std::string& save_path, bool to_reference_frame,
                    bool add_frame) const {
  if (!boost::filesystem::exists(save_path)) {
    ROS_ERROR("Cannot save cloud, directory does not exist: %s",
              save_path.c_str());
    return;
  }

  std::string file_name_prefix = save_path + std::to_string(stamp_.toSec());
  if (!to_reference_frame) {
    pcl::io::savePCDFileASCII(file_name_prefix + ".pcd", pointcloud_);
    return;
  }

  PointCloud cloud_initial;
  Eigen::Matrix4d T_REFFRAME_LIDAR_initial =
      T_REFFRAME_BASELINK_initial_ * T_BASELINK_LIDAR_;
  pcl::transformPointCloud(pointcloud_, cloud_initial,
                           T_REFFRAME_LIDAR_initial);

  PointCloud cloud_final;
  Eigen::Matrix4d T_REFFRAME_LIDAR_final =
      T_REFFRAME_BASELINK() * T_BASELINK_LIDAR_;
  pcl::transformPointCloud(pointcloud_, cloud_final, T_REFFRAME_LIDAR_final);

  PointCloudCol cloud_initial_col =
      beam::ColorPointCloud(cloud_initial, 255, 0, 0);
  PointCloudCol cloud_final_col = beam::ColorPointCloud(cloud_final, 0, 255, 0);

  if (add_frame) {
    cloud_initial_col =
        beam::AddFrameToCloud(cloud_initial_col, T_REFFRAME_LIDAR_initial);
    cloud_final_col =
        beam::AddFrameToCloud(cloud_final_col, T_REFFRAME_LIDAR_final);
  }

  pcl::io::savePCDFileASCII(file_name_prefix + "_initial.pcd",
                            cloud_initial_col);
  pcl::io::savePCDFileASCII(file_name_prefix + "_final.pcd", cloud_final_col);

  ROS_INFO("Saved cloud with stamp: %.5f", stamp_.toSec());
}

void ScanPose::SaveLoamCloud(const std::string& save_path,
                             bool to_reference_frame, bool add_frame) const {
  if (cloud_type_ != "LOAMPOINTCLOUD") {
    ROS_WARN("Scan pose has no LOAM pointcloud, not saving cloud.");
    return;
  }

  if (!boost::filesystem::exists(save_path)) {
    ROS_ERROR("Cannot save cloud, directory does not exist: %s",
              save_path.c_str());
    return;
  }

  std::string file_name_prefix = save_path + std::to_string(stamp_.toSec());
  boost::filesystem::create_directories(file_name_prefix);
  if (!to_reference_frame) {
    loampointcloud_.Save(file_name_prefix, true);
    return;
  }

  Eigen::Matrix4d T_REFFRAME_LIDAR_final =
      T_REFFRAME_BASELINK() * T_BASELINK_LIDAR_;
  beam_matching::LoamPointCloud loam_cloud_transformed = loampointcloud_;
  loam_cloud_transformed.TransformPointCloud(T_REFFRAME_LIDAR_final);
  loam_cloud_transformed.Save(file_name_prefix, true);
}

}  // namespace bs_common