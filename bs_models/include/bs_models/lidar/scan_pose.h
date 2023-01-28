#pragma once

#include <fuse_core/graph.h>
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>

#include <beam_matching/loam/LoamFeatureExtractor.h>
#include <beam_utils/pointclouds.h>

namespace bs_models {

/**
 * @brief class to store scan information along with its pose. The pose is
 * always stored as a transform from the baselink to some reference frame
 * (usually world, or submap). The scan data is always stored in the lidar frame
 * for two reasons: (1) saves us from needlessly transforming all points, (2)
 * allows us to use these measurements for extrinsic calibration. The poses are
 * stored in the baselink frame to make SLAM easier with fuse. All post
 * variables in the graph will always be relative to the baselink
 */
class ScanPose {
public:
  /**
   * @brief constructor for when inputting a regular pointcloud (not loam cloud)
   * @param cloud input pointcloud of type pcl::PointCloud<pcl::PointXYZ>> (this
   * cannot be a loam cloud)
   * @param stamp timestamp for this scan frame
   * @param T_REFFRAME_BASELINK transformation from the baselink frame to the
   * reference frame
   * @param T_BASELINK_LIDAR optional extrinsics between baselink and lidar. If
   * not set, it will assume the baselink is the lidar frame so this transform
   * will be set to identity.
   * @param feature_extractor optional loam feature extractor. If supplied, the
   * constructor will extract loam features and store it in this class
   */
  ScanPose(
      const PointCloud& cloud, const ros::Time& stamp,
      const Eigen::Matrix4d& T_REFFRAME_BASELINK,
      const Eigen::Matrix4d& T_BASELINK_LIDAR = Eigen::Matrix4d::Identity(),
      const std::shared_ptr<beam_matching::LoamFeatureExtractor>&
          feature_extractor = nullptr);

  /**
   * @brief constructor for when inputting a PointXYZIRT pointcloud
   * (velodyne pointcloud)
   * @param cloud input pointcloud of type pcl::PointCloud<PointXYZIRT>>
   * @param stamp timestamp for this scan frame
   * @param T_REFFRAME_BASELINK transformation from the baselink frame to the
   * reference frame
   * @param T_BASELINK_LIDAR optional extrinsics between baselink and lidar. If
   * not set, it will assume the baselink is the lidar frame so this transform
   * will be set to identity.
   * @param feature_extractor optional loam feature extractor. If supplied, the
   * constructor will extract loam features and store it in this class
   */
  ScanPose(
      const pcl::PointCloud<PointXYZIRT>& cloud, const ros::Time& stamp,
      const Eigen::Matrix4d& T_REFFRAME_BASELINK,
      const Eigen::Matrix4d& T_BASELINK_LIDAR = Eigen::Matrix4d::Identity(),
      const std::shared_ptr<beam_matching::LoamFeatureExtractor>&
          feature_extractor = nullptr);

  /**
   * @brief constructor for when inputting a PointXYZITRRNR pointcloud
   * (ouster pointcloud)
   * @param cloud input pointcloud of type
   * pcl::PointCloud<PointXYZITRRNR>>
   * @param stamp timestamp for this scan frame
   * @param T_REFFRAME_BASELINK transformation from the baselink frame to the
   * reference frame
   * @param T_BASELINK_LIDAR optional extrinsics between baselink and lidar. If
   * not set, it will assume the baselink is the lidar frame so this transform
   * will be set to identity.
   * @param feature_extractor optional loam feature extractor. If supplied, the
   * constructor will extract loam features and store it in this class
   */
  ScanPose(
      const pcl::PointCloud<PointXYZITRRNR>& cloud, const ros::Time& stamp,
      const Eigen::Matrix4d& T_REFFRAME_BASELINK,
      const Eigen::Matrix4d& T_BASELINK_LIDAR = Eigen::Matrix4d::Identity(),
      const std::shared_ptr<beam_matching::LoamFeatureExtractor>&
          feature_extractor = nullptr);

  /**
   * @brief constructor that does not require an input cloud. YOU WILL NEED TO
   * MANUALLY ADD CLOUDS AFTER INSTANTIATION USING: AddPointCloud(...) and/or
   * AddLoamCloud(...)
   * @param stamp timestamp for this scan frame
   * @param T_REFFRAME_BASELINK transformation from the cloud (scan frame) to
   * the reference frame (usually WORLD or SUBMAP)
   * @param T_BASELINK_LIDAR optional extrinsics between baselink and lidar. If
   * not set, it will assume the baselink is the lidar frame so this transform
   * will be set to identity.
   */
  ScanPose(
      const ros::Time& stamp, const Eigen::Matrix4d& T_REFFRAME_BASELINK,
      const Eigen::Matrix4d& T_BASELINK_LIDAR = Eigen::Matrix4d::Identity());

  /**
   * @brief add regular pointcloud
   * @param cloud input pointcloud of type pcl::PointCloud<pcl::PointXYZ>> (this
   * cannot be a loam cloud), where points are expressed in lidar frame
   * @param override_cloud whether or not to override the cloud, otherwise it
   * will add to the cloud.
   */
  void AddPointCloud(const PointCloud& cloud, bool override_cloud = false);

  /**
   * @brief add loam pointcloud
   * @param cloud input pointcloud of type LoamPointCloud, where points are
   * expressed in lidar frame
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
  bool UpdatePose(const fuse_core::Graph::ConstSharedPtr& graph_msg);

  /**
   * @brief update the pose of this ScanPose given a transformation matrix
   * @param T_REFFRAME_BASELINK transformation from the lidar frame to
   * the reference frame (usually WORLD or SUBMAP)
   */
  void UpdatePose(const Eigen::Matrix4d& T_REFFRAME_BASELINK);

  /**
   * @brief update the extrinsics of this ScanPose given a transformation matrix
   * @param T_BASELINK_LIDAR transformation from the lidar frame to
   * the baselink frame
   */
  void UpdateExtrinsics(const Eigen::Matrix4d& T_BASELINK_LIDAR);

  /**
   * @brief check if this scanpose is near some some other scan pose in the time
   * domain
   * @param time query time
   * @param tolerance max time difference for this to return true
   * @return true if difference is less than query time
   */
  bool Near(const ros::Time& time, const double tolerance) const;

  /**
   * @brief get the number of times this scanpose has has its pose updated
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
   * @brief return the current estimate of the position (t_REFFREAME_BASELINK)
   * @return position fuse variable
   */
  fuse_variables::Position3DStamped Position() const;

  /**
   * @brief return the current estimate of the orientation
   * (R_REFFREAME_BASELINK)
   * @return orientation fuse variable
   */
  fuse_variables::Orientation3DStamped Orientation() const;

  /**
   * @brief return the current pose of this scanpose.
   * @return T_REFFRAME_BASELINK transformation from the baselink frame to
   * the reference frame  (usually WORLD or SUBMAP)
   */
  Eigen::Matrix4d T_REFFRAME_BASELINK() const;

  /**
   * @brief return the current pose of the lidar relative to the reference frame
   * @return T_REFFRAME_LIDAR transformation from the lidar frame to
   * the reference frame  (usually WORLD or SUBMAP)
   */
  Eigen::Matrix4d T_REFFRAME_LIDAR() const;

  /**
   * @brief return the initial estimate of the pose of this scanpose.
   * @return T_REFFRAME_BASELINK_INIT transformation from the baselink frame
   * to the reference frame  (usually WORLD or SUBMAP)
   */
  const Eigen::Matrix4d T_REFFRAME_BASELINK_INIT() const;

  /**
   * @brief return the original estimate of the pose of the lidar relative to
   * the reference frame
   * @return T_REFFRAME_LIDAR_init transformation from the lidar frame to
   * the reference frame  (usually WORLD or SUBMAP)
   */
  Eigen::Matrix4d T_REFFRAME_LIDAR_INIT() const;

  /**
   * @brief get the extrinsics of this ScanPose
   * @return T_BASELINK_LIDAR transformation from the lidar frame to
   * the baselink frame
   */
  Eigen::Matrix4d T_BASELINK_LIDAR() const;

  /**
   * @brief get the extrinsics of this ScanPose
   * @return T_LIDAR_BASELINK transformation from the baselink to the lidar
   * frame
   */
  Eigen::Matrix4d T_LIDAR_BASELINK() const;

  /**
   * @brief return regular cloud (not loam)
   * @return cloud, where points are expressed in the lidar frame
   */
  PointCloud Cloud() const;
  /**
   * @brief return loam pointcloud
   * @return cloud, where points are expressed in the lidar frame
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
   * @brief sets the cloud type to LOAMPOINTCLOUD
   */
  void SetCloudTypeToLoam();

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
  void SaveCloud(const std::string& save_path, bool to_reference_frame = true,
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

  /**
   * @brief save all scan pose data to an output directory in a way that can be
   * re-loaded using LoadData(). Output format:
   *
   *  /output_dir/
   *    scan_pose.json (general data)
   *    pointcloud.pcd
   *    loam_edges_strong.pcd
   *    loam_edges_weak.pcd
   *    loam_surfaces_strong.pcd
   *    loam_surfaces_weak.pcd
   *
   *
   * @param output_dir full path to empty directory. This path must exist, but
   * must be empty.
   */
  void SaveData(const std::string& output_dir) const;

  /**
   * @brief load data from some root directory which has the data saved in
   * the format that is output by SaveData (see above).
   * @param root_dir root directory which has all data to be loaded. Note
   * that this can only contain ScanPose data for one object.
   * @return true if successful
   */
  bool LoadData(const std::string& root_dir);

protected:
  // pose data
  ros::Time stamp_;
  int updates_{0};
  fuse_variables::Position3DStamped position_;
  fuse_variables::Orientation3DStamped orientation_;
  Eigen::Matrix4d T_REFFRAME_BASELINK_initial_;
  Eigen::Matrix4d T_BASELINK_LIDAR_;

  // cloud data: all in lidar frame
  PointCloud pointcloud_;
  beam_matching::LoamPointCloud loampointcloud_;

  /** This is mainly used to determine if the loam pointcloud is polutated or
   * not. If so, we can run loam scan registration. Options: PCLPOINTCLOUD,
   * LOAMPOINTCLOUD */
  std::string cloud_type_{"PCLPOINTCLOUD"};
};

} // namespace bs_models
