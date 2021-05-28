#include <global_mapping/submap.h>

#include <beam_common/utils.h>

namespace global_mapping {

Submap::Submap(const ros::Time& stamp, const Eigen::Matrix4d& T_WORLD_SUBMAP,
               const std::shared_ptr<ExtrinsicsLookup>& extrinsics)
    : stamp_(stamp), extrinsics_(extrinsics) {
  // create fuse variables
  position_ = fuse_variables::Position3DStamped(stamp, fuse_core::uuid::NIL);
  orientation_ =
      fuse_variables::Orientation3DStamped(stamp, fuse_core::uuid::NIL);

  // add transform
  beam_common::EigenTransformToFusePose(T_WORLD_SUBMAP, position_,
                                        orientation_);
}

Submap::Submap(const ros::Time& stamp,
               const fuse_variables::Position3DStamped& position,
               const fuse_variables::Orientation3DStamped& orientation,
               const std::shared_ptr<ExtrinsicsLookup>& extrinsics)
    : position_(position),
      orientation_(orientation),
      stamp_(stamp),
      extrinsics_(extrinsics) {}

fuse_variables::Position3DStamped Submap::Position() const { return position_; }

fuse_variables::Orientation3DStamped Submap::Orientation() const {
  return orientation_;
}

Eigen::Matrix4d Submap::T_WORLD_SUBMAP() const {
  return beam_common::FusePoseToEigenTransform(position_, orientation_);
}

Eigen::Matrix4d Submap::T_WORLD_SUBMAP_INIT() const {
  return T_WORLD_SUBMAP_initial_;
}

int Submap::Updates() const { return graph_updates_; }

ros::Time Submap::Stamp() const { return stamp_; }

void Submap::AddCameraMeasurement(
    const std::vector<LandmarkMeasurementMsg>& landmarks,
    const Eigen::Matrix4d& T_WORLD_FRAME, const ros::Time& stamp, int sensor_id,
    int measurement_id) {
  // TODO: convert to proper submap frame
}

void Submap::AddLidarMeasurement(const PointCloud& cloud,
                                 const Eigen::Matrix4d& T_WORLD_FRAME,
                                 const ros::Time& stamp, int sensor_id,
                                 int measurement_id, int type) {
  // TODO: convert to proper submap frame

  // Check if stamp already exists (we may be adding partial scans)
  auto iter = lidar_keyframe_poses_.find(stamp.toNSec());
  if (iter != lidar_keyframe_poses_.end()) {
    // Stamp exists: add cloud to the corresponding submap

    // if it's a normal cloud, we can just add it
    if (type == 0) {
      iter->second.AddPointCloud(cloud, false);
      return;
    }

    // if it's a loamcloud, we need to add to the correct type
    beam_matching::LoamPointCloud new_loam_cloud;
    if (type == 1) {
      new_loam_cloud.AddEdgeFeaturesStrong(cloud);
    } else if (type == 2) {
      new_loam_cloud.AddSurfaceFeaturesStrong(cloud);
    } else {
      BEAM_ERROR(
          "Invalid pointcloud type, not adding to submap. Input: {}, Options: "
          "0, 1, 2. See LidarMeasurement.msg for more information,",
          type);
      return;
    }
    iter->second.AddPointCloud(new_loam_cloud);
  } else {
    // Stamp does not exist: add new scanpose to map
    beam_common::ScanPose new_scan_pose(stamp, T_WORLD_FRAME);
    if (type == 0) {
      new_scan_pose.AddPointCloud(cloud, false);
      lidar_keyframe_poses_.insert(std::pair<uint64_t, beam_common::ScanPose>(
          stamp.toNSec(), new_scan_pose));
      return;
    }

    beam_matching::LoamPointCloud new_loam_cloud;
    if (type == 1) {
      new_loam_cloud.AddEdgeFeaturesStrong(cloud);
    } else if (type == 2) {
      new_loam_cloud.AddSurfaceFeaturesStrong(cloud);
    } else {
      BEAM_ERROR(
          "Invalid pointcloud type, not adding to submap. Input: {}, Options: "
          "0, 1, 2. See LidarMeasurement.msg for more information,",
          type);
      return;
    }
    new_scan_pose.AddPointCloud(new_loam_cloud, false);
    lidar_keyframe_poses_.insert(std::pair<uint64_t, beam_common::ScanPose>(
        stamp.toNSec(), new_scan_pose));
  }
}

void Submap::AddTrajectoryMeasurement(
    const std::vector<Eigen::Matrix4d, pose_allocator>& poses,
    const std::vector<ros::Time>& stamps, const Eigen::Matrix4d& T_WORLD_FRAME,
    const ros::Time& stamp, int sensor_id, int measurement_id) {
  // TODO: convert to proper submap frame
}

bool Submap::UpdatePose(fuse_core::Graph::ConstSharedPtr graph_msg) {
  if (graph_msg->variableExists(position_.uuid()) &&
      graph_msg->variableExists(orientation_.uuid())) {
    position_ = dynamic_cast<const fuse_variables::Position3DStamped&>(
        graph_msg->getVariable(position_.uuid()));

    orientation_ = dynamic_cast<const fuse_variables::Orientation3DStamped&>(
        graph_msg->getVariable(orientation_.uuid()));
    graph_updates_++;
    return true;
  }
  return false;
}

bool Submap::Near(const ros::Time& time, const double tolerance) const {
  return (std::abs(stamp_.toSec() - time.toSec()) <= tolerance);
}

bool Submap::operator<(const Submap& rhs) const {
  return (stamp_ < rhs.stamp_);
}

void Submap::SaveKeypointsMapInWorldFrame(const std::string& filename) const {
  //
}

void Submap::SaveLidarMapInWorldFrame(const std::string& filename) const {
  //
}

PointCloud Submap::GetKeypointsInWorldFrame() const {
  //
}

PointCloud Submap::GetLidarPointsInWorldFrame() const {
  //
}

beam_matching::LoamPointCloud Submap::GetLidarLoamPointsInWorldFrame() const {
  //
}

void Submap::Print(std::ostream& stream) const {
  stream << "  Stamp: " << stamp_ << "\n"
         << "  Number of Updates: " << graph_updates_ << "\n"
         << "  Position:\n"
         << "  - x: " << position_.x() << "\n"
         << "  - y: " << position_.y() << "\n"
         << "  - z: " << position_.z() << "\n"
         << "  Orientation:\n"
         << "  - x: " << orientation_.x() << "\n"
         << "  - y: " << orientation_.y() << "\n"
         << "  - z: " << orientation_.z() << "\n"
         << "  - w: " << orientation_.w() << "\n";

  // TODO: add more info to the print (e.g. lidar and cam measurements)
}

}  // namespace global_mapping
