#pragma once

#include <list>

#include <fuse_core/fuse_macros.h>
#include <fuse_graphs/hash_graph.h>
#include <sensor_msgs/PointCloud2.h>

#include <beam_filtering/Utils.h>
#include <beam_matching/loam/LoamFeatureExtractor.h>
#include <beam_utils/pointclouds.h>

#include <bs_common/extrinsics_lookup_online.h>
#include <bs_models/lidar/scan_pose.h>
#include <bs_models/scan_registration/scan_to_map_registration.h>

namespace bs_models {

using LidarTransactionType = bs_constraints::Pose3DStampedTransaction;

class LidarPathInit {
public:
  explicit LidarPathInit(int lidar_buffer_size,
                         const std::string& matcher_config);

  void ProcessLidar(const sensor_msgs::PointCloud2::ConstPtr& msg);

  /**
   * @brief iterates through all keyframes in the list and add up the change in
   * position between each keyframe.
   * @return trajectory length
   */
  double CalculateTrajectoryLength() const;

  // initial path estimate for performing initialization, stored as
  // T_WORLD_BASELINK

  /**
   * @brief get the generated path
   * @return map timestamp in NS -> T_WORLD_BASELINK
   */
  std::map<uint64_t, Eigen::Matrix4d> GetPath() const;

  /**
   * @brief get the generated transactions for all the final keyframe
   * registrations against the map
   * @return map timestamp in NS -> transaction
   */
  std::map<uint64_t, LidarTransactionType> GetTransactions() const;

  /**
   * @brief Save three types of scans to separate folders:
   *
   * (1) scans transformed to world frame using imu pre-integration pose
   * estimate (2) scans transformed to world frame using loam refined pose
   * estimates (3) scans transformed to world frame using final pose estimates
   * from factor graph
   *
   */
  void OutputResults(const std::string& output_dir) const;

  /**
   * @brief use the output from a fuse graph to update the registration map.
   * This is useful because it better initializes the registration map for
   * future scan matching in the lidar odom. We also increase the registration
   * map to the size of all keyframes so that future scan registration has the
   * opportunity to use all available initial data
   */
  void UpdateRegistrationMap(fuse_core::Graph::ConstSharedPtr graph_msg);

  /**
   * @brief Removes all frames prior to start_time (unless it's not set), sets
   * the first scan pose in the keyframes list to identity, and adjusts all
   * subsequent poses to reflect this start change. The reason for this is that
   * the initial scan pose will be calculated with respect to the first
   * keyframe. Since we often rejects frames at the start that are stationary,
   * the post of the first kept keyframe likely won't be identity and is likely
   * to have drifted.
   */
  void SetTrajectoryStart(const ros::Time& start_time = ros::Time(0));

  double GetMaxRegistrationTimeInS();
  double GetMedianRegistrationTimeInS();
  double GetMeanRegistrationTimeInS();

private:
  bool InitExtrinsics(const ros::Time& stamp);

  Eigen::Matrix4d Get_T_WORLD_BASELINKEST(const ros::Time& stamp);

  void RemoveTransactionsWithStamp(const ros::Time& stamp);

  // scan registration objects
  std::unique_ptr<scan_registration::ScanToMapLoamRegistration>
      scan_registration_;
  std::shared_ptr<beam_matching::LoamFeatureExtractor> feature_extractor_;
  std::vector<beam_filtering::FilterParamsType> input_filter_params_;

  // store all current keyframes to be processed. Data in scan poses have
  // already been converted to the baselink frame, and T_BASELINK_LIDAR is set
  // to identity
  std::list<ScanPose> keyframes_;

  // store map from current keyframe timestamps in Ns to registration
  // transaction
  std::map<uint64_t, LidarTransactionType> keyframe_transactions_;

  // how many lidar scans to keep in the list of keyframes for calculating
  // length
  int lidar_buffer_size_;

  bs_common::ExtrinsicsLookupOnline& extrinsics_ =
      bs_common::ExtrinsicsLookupOnline::GetInstance();
  bool extrinsics_initialized_{false};
  Eigen::Matrix4d T_BASELINK_LIDAR_;
  std::set<double> registration_times_;

  // for outputting results
  PointCloud map_optimized_;
  PointCloud map_registered_;
  PointCloud map_init_;

  // params only tunable here
  bool forward_predict_{false};
  int min_spline_count_{8};
  bool skip_alternate_scans_{true};
  bool registered_last_{false};
};

} // namespace bs_models