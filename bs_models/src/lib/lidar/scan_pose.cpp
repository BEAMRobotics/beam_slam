#include <bs_models/lidar/scan_pose.h>

#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <beam_utils/filesystem.h>
#include <beam_utils/math.h>

#include <bs_common/utils.h>

namespace bs_models {

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

ScanPose::ScanPose(const pcl::PointCloud<PointXYZIRT>& cloud,
                   const ros::Time& stamp,
                   const Eigen::Matrix4d& T_REFFRAME_BASELINK,
                   const Eigen::Matrix4d& T_BASELINK_LIDAR,
                   const std::shared_ptr<beam_matching::LoamFeatureExtractor>&
                       feature_extractor)
    : stamp_(stamp),
      T_REFFRAME_BASELINK_initial_(T_REFFRAME_BASELINK),
      T_BASELINK_LIDAR_(T_BASELINK_LIDAR) {
  // convert to regular pointcloud
  for (const auto& p : cloud) {
    pointcloud_.push_back(pcl::PointXYZ(p.x, p.y, p.z));
  }

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

ScanPose::ScanPose(const pcl::PointCloud<PointXYZITRRNR>& cloud,
                   const ros::Time& stamp,
                   const Eigen::Matrix4d& T_REFFRAME_BASELINK,
                   const Eigen::Matrix4d& T_BASELINK_LIDAR,
                   const std::shared_ptr<beam_matching::LoamFeatureExtractor>&
                       feature_extractor)
    : stamp_(stamp),
      T_REFFRAME_BASELINK_initial_(T_REFFRAME_BASELINK),
      T_BASELINK_LIDAR_(T_BASELINK_LIDAR) {
  // convert to regular pointcloud
  for (const auto& p : cloud) {
    pointcloud_.push_back(pcl::PointXYZ(p.x, p.y, p.z));
  }

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
  position_ = fuse_variables::Position3DStamped(stamp_, fuse_core::uuid::NIL);
  orientation_ =
      fuse_variables::Orientation3DStamped(stamp_, fuse_core::uuid::NIL);

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
  cloud_type_ = "LOAMPOINTCLOUD";
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

int ScanPose::Updates() const {
  return updates_;
}

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

Eigen::Matrix4d ScanPose::T_BASELINK_LIDAR() const {
  return T_BASELINK_LIDAR_;
}

Eigen::Matrix4d ScanPose::T_LIDAR_BASELINK() const {
  return beam::InvertTransform(T_BASELINK_LIDAR_);
}

PointCloud ScanPose::Cloud() const {
  return pointcloud_;
}

beam_matching::LoamPointCloud ScanPose::LoamCloud() const {
  return loampointcloud_;
}

ros::Time ScanPose::Stamp() const {
  return stamp_;
}

std::string ScanPose::Type() const {
  return cloud_type_;
}

void ScanPose::SetCloudTypeToLoam() {
  cloud_type_ = "LOAMPOINTCLOUD";
}

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

void ScanPose::SaveData(const std::string& output_dir) const {
  if (!boost::filesystem::exists(output_dir)) {
    BEAM_ERROR("Invalid output directory, not saving ScanPose data. Input: {}",
               output_dir);
    return;
  }

  // save general data
  nlohmann::json J_scanpose = {
      {"stamp_nsecs", stamp_.toNSec()},
      {"updates", updates_},
      {"pointcloud_size", pointcloud_.size()},
      {"loam_edges_strong", loampointcloud_.edges.strong.cloud.size()},
      {"loam_surfaces_strong", loampointcloud_.surfaces.strong.cloud.size()},
      {"loam_edges_weak", loampointcloud_.edges.weak.cloud.size()},
      {"loam_surfaces_weak", loampointcloud_.surfaces.weak.cloud.size()},
      {"cloud_type", cloud_type_},
      {"device_id", fuse_core::uuid::to_string(position_.deviceId())},
      {"position_xyz", {position_.x(), position_.y(), position_.z()}},
      {"orientation_xyzw",
       {orientation_.x(), orientation_.y(), orientation_.z(),
        orientation_.w()}}};
  beam::AddTransformToJson(J_scanpose, T_BASELINK_LIDAR_, "T_BASELINK_LIDAR");
  beam::AddTransformToJson(J_scanpose, T_REFFRAME_BASELINK_initial_,
                           "T_REFFRAME_BASELINK_initial");
  std::string scanpose_filename = output_dir + "scan_pose.json";
  std::ofstream scanpose_file(scanpose_filename);
  scanpose_file << std::setw(4) << J_scanpose << std::endl;

  // save pointclouds
  std::string error_message;
  if (!beam::SavePointCloud<pcl::PointXYZ>(
          output_dir + "pointcloud.pcd", pointcloud_,
          beam::PointCloudFileType::PCDBINARY, error_message)) {
    BEAM_ERROR("Unable to save cloud. Reason: {}", error_message);
  }

  beam_matching::LoamPointCloudCombined c = loampointcloud_.GetCombinedCloud();
  if (!beam::SavePointCloud<PointLoam>(output_dir + "loam_cloud.pcd", c,
                                       beam::PointCloudFileType::PCDBINARY,
                                       error_message)) {
    BEAM_ERROR("Unable to save loam cloud. Reason: {}", error_message);
  }
}

bool ScanPose::LoadData(const std::string& root_dir) {
  if (!boost::filesystem::exists(root_dir)) {
    BEAM_ERROR("Invalid input directory, not loading scanpose data. Input: {}",
               root_dir);
    return false;
  }

  // load general data
  nlohmann::json J;
  std::ifstream file(root_dir + "scan_pose.json");
  file >> J;
  stamp_.fromNSec(J["stamp_nsecs"]);
  updates_ = J["updates"];
  std::string cloud_type_read = J["cloud_type"];
  if (cloud_type_read == "PCLPOINTCLOUD" ||
      cloud_type_read == "LOAMPOINTCLOUD") {
    cloud_type_ = cloud_type_read;
  } else {
    cloud_type_ = "PCLPOINTCLOUD";
  }

  // load position data
  position_ = fuse_variables::Position3DStamped(
      stamp_, fuse_core::uuid::from_string(J["device_id"]));
  std::vector<double> position_vector = J["position_xyz"];
  position_.x() = position_vector.at(0);
  position_.y() = position_vector.at(1);
  position_.z() = position_vector.at(2);

  orientation_ = fuse_variables::Orientation3DStamped(
      stamp_, fuse_core::uuid::from_string(J["device_id"]));
  std::vector<double> orientation_vector = J["orientation_xyzw"];
  orientation_.x() = orientation_vector.at(0);
  orientation_.y() = orientation_vector.at(1);
  orientation_.z() = orientation_vector.at(2);
  orientation_.w() = orientation_vector.at(3);

  std::vector<double> T_REFFRAME_BASELINK_initial_vec =
      J["T_REFFRAME_BASELINK_initial"];
  T_REFFRAME_BASELINK_initial_ =
      beam::VectorToEigenTransform(T_REFFRAME_BASELINK_initial_vec);

  std::vector<double> T_BASELINK_LIDAR_vec = J["T_BASELINK_LIDAR"];
  T_BASELINK_LIDAR_ = beam::VectorToEigenTransform(T_BASELINK_LIDAR_vec);

  // load pointclouds
  std::string pointcloud_filename;
  pointcloud_filename = root_dir + "pointcloud.pcd";
  if (boost::filesystem::exists(pointcloud_filename)) {
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pointcloud_filename, pointcloud_) ==
        -1) {
      BEAM_ERROR("Couldn't read pointcloud file: {}", pointcloud_filename);
      return false;
    }
  }

  pointcloud_filename = root_dir + "loam_cloud.pcd";
  if (boost::filesystem::exists(pointcloud_filename)) {
    beam_matching::LoamPointCloudCombined loam_combined;
    if (pcl::io::loadPCDFile<PointLoam>(pointcloud_filename, loam_combined) ==
        -1) {
      BEAM_ERROR("Couldn't read pointcloud file: {}", pointcloud_filename);
    }
    loampointcloud_.LoadFromCombined(loam_combined);
  }

  if (loampointcloud_.edges.strong.cloud.size() > 0 ||
      loampointcloud_.edges.weak.cloud.size() > 0 ||
      loampointcloud_.surfaces.strong.cloud.size() > 0 ||
      loampointcloud_.surfaces.weak.cloud.size() > 0) {
    cloud_type_ = "PCLPOINTCLOUD";
  }

  return true;
}

void ScanPose::SaveCloud(const std::string& save_path, bool to_reference_frame,
                         bool add_frame) const {
  if (!boost::filesystem::exists(save_path)) {
    ROS_ERROR("Cannot save cloud, directory does not exist: %s",
              save_path.c_str());
    return;
  }

  if (!to_reference_frame) {
    std::string filename = std::to_string(stamp_.toSec()) + ".pcd";
    std::string save_path = beam::CombinePaths(save_path, filename);
    std::string error_message;
    if (!beam::SavePointCloud<pcl::PointXYZ>(
            save_path, pointcloud_, beam::PointCloudFileType::PCDBINARY,
            error_message)) {
      BEAM_ERROR("Unable to save cloud. Reason: {}", error_message);
    }
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

  std::string filename = std::to_string(stamp_.toSec()) + ".pcd";
  std::string save_path_init =
      beam::CombinePaths(save_path, "initial_" + filename);
  std::string error_message{};
  if (!beam::SavePointCloud<pcl::PointXYZRGB>(
          save_path_init, cloud_initial_col,
          beam::PointCloudFileType::PCDBINARY, error_message)) {
    BEAM_ERROR("Unable to save cloud. Reason: {}", error_message);
  }
  std::string save_path_final =
      beam::CombinePaths(save_path, "final_" + filename);
  if (!beam::SavePointCloud<pcl::PointXYZRGB>(
          save_path_final, cloud_final_col, beam::PointCloudFileType::PCDBINARY,
          error_message)) {
    BEAM_ERROR("Unable to save cloud. Reason: {}", error_message);
  }
  if (error_message.empty()) {
    ROS_INFO("Saved cloud with stamp: %.5f", stamp_.toSec());
  }
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

  std::string cloud_dir =
      beam::CombinePaths(save_path, std::to_string(stamp_.toSec()));
  boost::filesystem::create_directory(cloud_dir);
  if (!to_reference_frame) {
    loampointcloud_.Save(cloud_dir, true);
    return;
  }

  Eigen::Matrix4d T_REFFRAME_LIDAR_final =
      T_REFFRAME_BASELINK() * T_BASELINK_LIDAR_;
  beam_matching::LoamPointCloud loam_cloud_transformed = loampointcloud_;
  loam_cloud_transformed.TransformPointCloud(T_REFFRAME_LIDAR_final);
  loam_cloud_transformed.Save(cloud_dir, true);
}

} // namespace bs_models