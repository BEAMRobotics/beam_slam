#pragma once

#include <list>

#include <sensor_msgs/PointCloud2.h>
#include <fuse_core/async_sensor_model.h>
#include <fuse_graphs/hash_graph.h>

#include <beam_utils/pointclouds.h>
#include <beam_filtering/Utils.h>
#include <beam_matching/Matchers.h>
#include <beam_matching/loam/LoamPointCloud.h>
#include <beam_matching/loam/LoamFeatureExtractor.h>

#include <bs_parameters/models/lo_initializer_params.h>
#include <bs_models/scan_registration/scan_to_map_registration.h>
#include <bs_common/extrinsics_lookup_online.h>
#include <bs_models/scan_pose.h>

namespace bs_models {

/**
 * @brief This initializer creates an initial trajectory using imu and lidar
 * data, while also initializing imu params including gravity directiona and
 * biases. It then publishes the results over ROS. The goal of this initializer
 * is to get a decent trajectory estimate and imu params (optional) that can be
 * used to begin either VIO or LIO. Either VIO or LIO may further refine the
 * poses and IMU params, but this should give a reliable first estimate with
 * scale.
 *
 * NOTE: this has been implemented as a sensor model for ease of use, however,
 * it does not access the fuse graph in any way since the estimation done in the
 * initializer neesd to be de-coupled from the graph used by the main SLAM.
 */
class LoInitializer : public fuse_core::AsyncSensorModel {
 public:
  SMART_PTR_DEFINITIONS(LoInitializer);

  LoInitializer();

  ~LoInitializer() override = default;

  /**
   * @brief Callback for lidar points processing, this callback has most of the
   * intializer implementation
   * @param[in] msg - The lidar message to process
   */
  void processLidar(const sensor_msgs::PointCloud2::ConstPtr& msg);

 protected:
  /**
   * @brief loads all params and sets up scan registration and feature extractor
   */
  void onInit() override;

  /**
   * @brief Register scan against previous and then add to the queue of scan
   * poses. After processing the keyframe, it will check if we are ready to
   * start optimizing. To start optimizing, we check that the time window is
   * greater than the time_window param, and that the trajectory length is
   * greater than min_trajectory_distance. If the time window is larger than the
   * min, but the trajectory is not long enough, throw away the first scan in
   * the queue and continue collecting keyframes
   */
  void ProcessCurrentKeyframe(const ros::Time& time);

  /**
   * @brief Sets the first scan pose in the keyframes list to identity, and
   * adjusts all subsequent poses to reflect this start change. The reason for
   * this is that the initial scan pose will be calculated with respect to the
   * first keyframe. Since we often rejects frames at the start that are
   * stationary, the post of the first kept keyframe likely won't be identity
   * and is likely to have drifted.
   */
  void SetTrajectoryStart();

  /**
   * @brief Save three types of scans to separate folders:
   *
   * (1) scans transformed to world frame using imu pre-integration pose
   * estimate (2) scans transformed to world frame using loam refined pose
   * estimates (3) scans transformed to world frame using final pose estimates
   * from factor graph
   *
   */
  void OutputResults();

  /**
   * @brief publish results of initialization to a InitializedPathMsg
   */
  void PublishResults();

  /**
   * @brief iterates through all keypoints in the list and add up the change in
   * position between each keyframe.
   * @param keyframes list of scan poses that makeup the trajectory of keyframes
   * @return trajectory length
   */
  double CalculateTrajectoryLength(const std::list<ScanPose>& keyframes);

  // subscribers & publishers
  ros::Subscriber lidar_subscriber_;
  ros::Publisher results_publisher_;

  // main parameters
  bs_parameters::models::LoInitializerParams params_;

  // get access to extrinsics singleton
  bs_common::ExtrinsicsLookupOnline& extrinsics_ =
      bs_common::ExtrinsicsLookupOnline::GetInstance();

  // scan registration objects
  std::unique_ptr<scan_registration::ScanToMapLoamRegistration>
      scan_registration_;
  std::shared_ptr<beam_matching::LoamFeatureExtractor> feature_extractor_;

  std::vector<beam_filtering::FilterParamsType> input_filter_params_;

  // store all current keyframes to be processed. Data in scan poses have
  // already been converted to the baselink frame, and T_BASELINK_LIDAR is set
  // to identity
  std::list<ScanPose> keyframes_;

  // keep track of the current keyframe
  int keyframe_scan_counter_{0};
  ros::Time keyframe_start_time_{0};
  ros::Time prev_stamp_{0};
  PointCloud keyframe_cloud_;
  Eigen::Matrix4d T_WORLD_KEYFRAME_{Eigen::Matrix4d::Identity()};

  // bool for tracking if initialization has completed
  bool initialization_complete_{false};
};

}  // namespace bs_models
