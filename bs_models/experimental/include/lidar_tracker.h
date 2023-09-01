#pragma once

#include <unordered_map>

#include <fuse_core/async_sensor_model.h>
#include <fuse_core/fuse_macros.h>
#include <fuse_core/throttled_callback.h>
#include <fuse_core/uuid.h>
#include <tf/transform_broadcaster.h>

#include <beam_filtering/Utils.h>
#include <beam_matching/Matchers.h>
#include <beam_utils/pointclouds.h>

#include <bs_common/extrinsics_lookup_online.h>
#include <bs_models/frame_initializers/frame_initializers.h>
#include <bs_models/global_mapping/active_submap.h>
#include <bs_models/lidar/scan_pose.h>
#include <bs_models/scan_registration/scan_registration_base.h>

#include "parameters/models/lidar_tracker_params.h"

namespace bs_models {

/**
 * @brief THIS CLASS NEEDS TO BE REDONE
 *
 * This file is just a copy of LidarOdometry before we removed the global
 * registration stuff. We need to remove all local registration stuff, then test
 *
 */
class LidarTracker : public fuse_core::AsyncSensorModel {
public:
  FUSE_SMART_PTR_DEFINITIONS(LidarTracker);

  LidarTracker();

  ~LidarTracker() override = default;

private:
  void onStart() override;

  void onInit() override;

  void onStop() override;

  void onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph_msg) override;

  void process(const sensor_msgs::PointCloud2::ConstPtr& msg);

  fuse_core::Transaction::SharedPtr
      GenerateTransaction(const sensor_msgs::PointCloud2::ConstPtr& msg);

  void SetupRegistration();

  fuse_core::Transaction::SharedPtr
      RegisterScanToGlobalMap(const ScanPose& scan_pose,
                              Eigen::Matrix4d& T_WORLD_BASELINK);

  void SendRelocRequest(const std::shared_ptr<ScanPose>& scan_pose);

  void PublishMarginalizedScanPose(const std::shared_ptr<ScanPose>& scan_pose);

  void SaveMarginalizedScanPose(const std::shared_ptr<ScanPose>& scan_pose);

  void PublishScanRegistrationResults(
      const fuse_core::Transaction::SharedPtr& transaction_lm,
      const fuse_core::Transaction::SharedPtr& transaction_gm,
      const ScanPose& scan_pose);

  void PublishTfTransform(const Eigen::Matrix4d& T_CHILD_PARENT,
                          const std::string& child_frame,
                          const std::string& parent_frame,
                          const ros::Time& time);

  /** subscribe to lidar data */
  ros::Subscriber subscriber_;

  /** Publishers */
  ros::Publisher results_publisher_;       // for global mapper
  ros::Publisher reloc_request_publisher_; // for global mapper
  ros::Publisher registration_publisher_init_;
  ros::Publisher registration_publisher_aligned_lm_;
  ros::Publisher registration_publisher_aligned_gm_;

  ros::Publisher odom_publisher_smooth_;
  int odom_publisher_smooth_counter_{0};
  ros::Publisher odom_publisher_global_;
  int odom_publisher_global_counter_{0};
  ros::Publisher odom_publisher_marginalized_;
  int odom_publisher_marginalized_counter_{0};

  tf::TransformBroadcaster tf_broadcaster_;

  int published_registration_results_{0};

  /** callback for lidar data */
  using ThrottledCallback =
      fuse_core::ThrottledMessageCallback<sensor_msgs::PointCloud2>;
  ThrottledCallback throttled_callback_;

  /** Needed for outputing the slam results or saving final clouds or graph
   * updates */
  std::list<std::shared_ptr<ScanPose>> active_clouds_;

  /** Get access to the active submap being published by the global mapper if
   * running */
  ActiveSubmap& active_submap_ = ActiveSubmap::GetInstance();

  /** Only needed if using LoamMatcher */
  std::shared_ptr<beam_matching::LoamFeatureExtractor> feature_extractor_;

  // register scans to local map
  std::unique_ptr<scan_registration::ScanRegistrationBase>
      local_scan_registration_;

  std::unique_ptr<PointcloudMatcher> global_matching_;
  std::unique_ptr<LoamMatcher> global_loam_matching_;

  fuse_core::UUID device_id_; //!< The UUID of this device

  /** Used to get initial pose estimates */
  std::unique_ptr<frame_initializers::FrameInitializerBase> frame_initializer_;

  bs_common::ExtrinsicsLookupOnline& extrinsics_ =
      bs_common::ExtrinsicsLookupOnline::GetInstance();

  bs_parameters::models::LidarTrackerParams params_;

  std::vector<beam_filtering::FilterParamsType> input_filter_params_;

  int updates_{0};
  ros::Duration reloc_request_period_;
  ros::Time last_reloc_request_time_{ros::Time(0)};
  Eigen::Matrix4d T_WORLD_BASELINKLAST_{Eigen::Matrix4d::Identity()};
  ros::Time last_scan_pose_time_{ros::Time(0)};
  std::string graph_updates_path_;
  std::string marginalized_scans_path_;
  std::string registration_results_path_;

  /** Params that can only be updated here: */
  bool update_local_map_on_graph_update_{false};
  bool use_frame_init_relative_{true};
};

} // namespace bs_models
