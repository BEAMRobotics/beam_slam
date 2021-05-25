#include <beam_common/scan_pose.h>

#include <boost/filesystem.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <beam_utils/math.h>

#include <beam_common/utils.h>

namespace beam_common {

ScanPose::ScanPose(const ros::Time& time,
                   const Eigen::Matrix4d& T_REFFRAME_CLOUD,
                   const PointCloud& cloud,
                   const std::shared_ptr<beam_matching::LoamFeatureExtractor>&
                       feature_extractor)
    : stamp_(time),
      pointcloud_(cloud),
      T_REFFRAME_CLOUD_initial_(T_REFFRAME_CLOUD) {
  // create fuse variables
  position_ = fuse_variables::Position3DStamped(time, fuse_core::uuid::NIL);
  orientation_ =
      fuse_variables::Orientation3DStamped(time, fuse_core::uuid::NIL);

  // add transform
  beam_common::EigenTransformToFusePose(T_REFFRAME_CLOUD, position_,
                                        orientation_);

  if (feature_extractor != nullptr) {
    cloud_type_ = "LOAMPOINTCLOUD";
    loampointcloud_ = feature_extractor->ExtractFeatures(cloud);
  }
}

ScanPose::ScanPose(const ros::Time& time,
                   const Eigen::Matrix4d& T_REFFRAME_CLOUD)
    : stamp_(time), T_REFFRAME_CLOUD_initial_(T_REFFRAME_CLOUD) {
  // create fuse variables
  position_ = fuse_variables::Position3DStamped(time, fuse_core::uuid::NIL);
  orientation_ =
      fuse_variables::Orientation3DStamped(time, fuse_core::uuid::NIL);

  // add transform
  beam_common::EigenTransformToFusePose(T_REFFRAME_CLOUD, position_,
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

bool ScanPose::Update(const fuse_core::Graph::ConstSharedPtr& graph_msg) {
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

Eigen::Matrix4d ScanPose::T_REFFRAME_CLOUD() const {
  Eigen::Matrix4d T_REFFRAME_CLOUD{Eigen::Matrix4d::Identity()};
  Eigen::Quaterniond q(orientation_.w(), orientation_.x(), orientation_.y(),
                       orientation_.z());
  T_REFFRAME_CLOUD.block(0, 3, 3, 1) =
      Eigen::Vector3d{position_.x(), position_.y(), position_.z()};
  T_REFFRAME_CLOUD.block(0, 0, 3, 3) = q.toRotationMatrix();
  return T_REFFRAME_CLOUD;
}

const Eigen::Matrix4d ScanPose::T_REFFRAME_CLOUD_INIT() const {
  return T_REFFRAME_CLOUD_initial_;
}

PointCloud ScanPose::Cloud() const { return pointcloud_; }

beam_matching::LoamPointCloud ScanPose::LoamCloud() const {
  return loampointcloud_;
}

ros::Time ScanPose::Stamp() const { return stamp_; }

std::string ScanPose::Type() const { return cloud_type_; }

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
                    bool add_frame) {
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
  PointCloud cloud_final;
  pcl::transformPointCloud(pointcloud_, cloud_initial,
                           T_REFFRAME_CLOUD_initial_);
  pcl::transformPointCloud(pointcloud_, cloud_final, this->T_REFFRAME_CLOUD());

  PointCloudCol cloud_initial_col =
      beam::ColorPointCloud(cloud_initial, 255, 0, 0);
  PointCloudCol cloud_final_col = beam::ColorPointCloud(cloud_final, 0, 255, 0);

  if (add_frame) {
    cloud_initial_col =
        beam::AddFrameToCloud(cloud_initial_col, T_REFFRAME_CLOUD_initial_);
    cloud_final_col =
        beam::AddFrameToCloud(cloud_final_col, this->T_REFFRAME_CLOUD());
  }
  pcl::io::savePCDFileASCII(file_name_prefix + "_initial.pcd",
                            cloud_initial_col);
  pcl::io::savePCDFileASCII(file_name_prefix + "_final.pcd", cloud_final_col);

  ROS_INFO("Saved cloud with stamp: %.5f", stamp_.toSec());
}

void ScanPose::SaveLoamCloud(const std::string& save_path,
                             bool to_reference_frame, bool add_frame) {
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

  beam_matching::LoamPointCloud loam_cloud_transformed = loampointcloud_;
  loam_cloud_transformed.TransformPointCloud(this->T_REFFRAME_CLOUD());
  loam_cloud_transformed.Save(file_name_prefix, true);
}

}  // namespace beam_common