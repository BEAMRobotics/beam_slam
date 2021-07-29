#pragma once

#include <fuse_core/graph.h>
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>

#include <beam_utils/pointclouds.h>
#include <beam_matching/loam/LoamFeatureExtractor.h>

namespace bs_common {

class ScanPose {
 public:
  ScanPose() = delete;

  /**
   * @brief constructor for when inputting a regular pointcloud (not loam cloud)
   * @param time timestamp for this scan frame
   * @param T_REFFRAME_CLOUD transformation from the cloud (scan frame) to the
   * reference frame
   * @param reference_frame_id frame that the pose is relative to (usually world or
   * submap frame)
   * @param cloud_frame_id frame that the pointcloud data is expressed in (usually
   * lidar or baselink)
   * @param cloud input pointcloud of type pcl::PointCloud<pcl::PointXYZ>> (this
   * cannot be a loam cloud)
   * @param feature_extractor if supplied, the constructor will extract loam
   * features and store it in this class
   */
  ScanPose(const ros::Time& time, const Eigen::Matrix4d& T_REFFRAME_CLOUD,
           const std::string& reference_frame_id, const std::string& cloud_frame_id,
           const PointCloud& cloud,
           const std::shared_ptr<beam_matching::LoamFeatureExtractor>&
               feature_extractor = nullptr);

  /**
   * @brief constructor that does not require an input cloud. YOU WILL NEED TO
   * MANUALLY ADD CLOUDS AFTER INSTANTIATION USING: AddPointCloud() and/or
   * AddLoamCloud
   * @param time timestamp for this scan frame
   * @param T_REFFRAME_CLOUD transformation from the cloud (scan frame) to the
   * reference frame (usually WORLD or SUBMAP)
   * @param reference_frame_id frame that the pose is relative to (usually world or
   * submap frame)
   * @param cloud_frame_id frame that the pointcloud data is expressed in (usually
   * lidar or baselink)
   */
  ScanPose(const ros::Time& time, const Eigen::Matrix4d& T_REFFRAME_CLOUD,
           const std::string& reference_frame_id, const std::string& cloud_frame_id);

  /**
   * @brief add regular pointcloud to this
   * @param cloud input pointcloud of type pcl::PointCloud<pcl::PointXYZ>> (this
   * cannot be a loam cloud)
   * @param override_cloud whether or not to override the cloud, otherwise it
   * will add to the cloud.
   */
  void AddPointCloud(const PointCloud& cloud, bool override_cloud = false);

  /**
   * @brief add regular pointcloud to this
   * @param cloud input pointcloud of type LoamPointCloud
   * @param override_cloud whether or not to override the cloud, otherwise it
   * will add to the cloud.
   */
  void AddPointCloud(const beam_matching::LoamPointCloud& cloud,
                     bool override_cloud = false);

  /**
   * @brief update the pose of this ScanPose given some graph message
   * @param graph_msg results from some optimizer which should contain the same
   * pose variable uuids that are stored herein
   * @return true update was successful (i.e., uuids were in the graph message)
   */
  bool Update(const fuse_core::Graph::ConstSharedPtr& graph_msg);

  /**
   * @brief update the pose of this ScanPose given a transformation matrix
   * @param T_REFFRAME_CLOUD transformation from the cloud (scan frame) to the
   * reference frame (usually WORLD or SUBMAP)
   */
  void Update(const Eigen::Matrix4d& T_REFFRAME_CLOUD);

  /**
   * @brief check if this scanpose is near some some other scan pose in the time
   * domain
   * @param time query time
   * @param tolerance max time difference for this to return true
   * @return true if difference is less than query time
   */
  bool Near(const ros::Time& time, const double tolerance) const;

  /**
   * @brief get the number of times this scanpose has has its pose updated by
   * some graph optimizer
   * @return number of pose updates
   */
  int Updates() const;

  /**
   * @brief check if this scanpose is less than (or occured before) some other
   * scan pose
   * @param rhs query scanpose
   * @return true if current time is less than input scanpose
   */
  bool operator<(const ScanPose& rhs) const;

  /**
   * @brief return the current estimate of the position
   * @return position fuse variable
   */
  fuse_variables::Position3DStamped Position() const;

  /**
   * @brief return the current estimate of the orientation
   * @return orientation fuse variable
   */
  fuse_variables::Orientation3DStamped Orientation() const;

  /**
   * @brief return the current pose of this scanpose.
   * @return T_REFFRAME_CLOUD transformation from the cloud (scan frame) to the
   * reference frame  (usually WORLD or SUBMAP)
   */
  Eigen::Matrix4d T_REFFRAME_CLOUD() const;

  /**
   * @brief return the initial estimate of the pose of this scanpose.
   * @return T_REFFRAME_CLOUD_INIT transformation from the cloud (scan frame) to
   * the reference frame  (usually WORLD or SUBMAP)
   */
  const Eigen::Matrix4d T_REFFRAME_CLOUD_INIT() const;

  /**
   * @brief return regular cloud (not loam)
   * @return cloud
   */
  PointCloud Cloud() const;
  /**
   * @brief return loam pointcloud
   * @return cloud
   */
  beam_matching::LoamPointCloud LoamCloud() const;

  /**
   * @brief return timestamp associated with this scanpose
   * @return stamp
   */
  ros::Time Stamp() const;

  /**
   * @brief return type of pointcloud.
   * @return type of cloud. Options: PCLPOINTCLOUD, LOAMPOINTCLOUD
   */
  std::string Type() const;

  /**
   * @brief return reference frame id
   * @return name of the reference frame, or the frame the the pose is relative to
   */
  std::string ReferenceFrameId() const;

    /**
   * @brief return the cloud frame id
   * @return name of the cloud frame, or the frame that all points are expressed in
   */
  std::string CloudFrameId() const;  

  /**
   * @brief print relevant information about what is currently contained in this
   * ScanPose.
   * @param stream input stream
   */
  void Print(std::ostream& stream = std::cout) const;

  /**
   * @brief save pointcloud of current scanpose
   * @param save_path full path to output directory. This directory must exist
   * @param to_reference_frame whether or not to confert to REFFRAME
   * @param add_frame whether or not to add coordinate frame to cloud
   */
  void Save(const std::string& save_path, bool to_reference_frame = true,
            bool add_frame = true) const;

  /**
   * @brief save loam pointcloud of current scanpose
   * @param save_path full path to output directory. This directory must exist
   * @param to_reference_frame whether or not to confert to REFFRAME
   * @param add_frame whether or not to add coordinate frame to cloud
   */
  void SaveLoamCloud(const std::string& save_path,
                     bool to_reference_frame = true,
                     bool add_frame = true) const;

 protected:
  ros::Time stamp_;
  int updates_{0};
  fuse_variables::Position3DStamped position_;
  fuse_variables::Orientation3DStamped orientation_;
  PointCloud pointcloud_;
  beam_matching::LoamPointCloud loampointcloud_;
  const Eigen::Matrix4d T_REFFRAME_CLOUD_initial_;
  std::string reference_frame_id_;
  std::string cloud_frame_id_;

  /** This is mainly used to determine if the loam pointcloud is polutated or
   * not. If so, we can run loam scan registration. Options: PCLPOINTCLOUD,
   * LOAMPOINTCLOUD */
  std::string cloud_type_{"PCLPOINTCLOUD"};
};

}  // namespace bs_common
