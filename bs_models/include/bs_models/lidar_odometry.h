#pragma once

#include <unordered_map>

#include <fuse_core/async_sensor_model.h>
#include <fuse_core/throttled_callback.h>
#include <fuse_core/uuid.h>

#include <beam_utils/pointclouds.h>
#include <beam_filtering/Utils.h>

#include <bs_constraints/relative_pose/relative_pose_transaction_base.h>
#include <bs_models/frame_initializers/frame_initializers.h>
#include <bs_models/scan_registration/scan_registration_base.h>
#include <bs_models/scan_pose.h>
#include <bs_models/active_submap.h>
#include <bs_common/extrinsics_lookup_online.h>
#include <bs_parameters/models/lidar_odometry_params.h>

namespace bs_models {

class LidarOdometry : public fuse_core::AsyncSensorModel {
 public:
  SMART_PTR_DEFINITIONS(LidarOdometry);

  LidarOdometry();

  ~LidarOdometry() override = default;

 private:
  void onStart() override;

  void onInit() override;

  void onStop() override;

  void onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph_msg) override;

  void process(const sensor_msgs::PointCloud2::ConstPtr& msg);

  bs_constraints::relative_pose::Pose3DStampedTransaction GenerateTransaction(
      const sensor_msgs::PointCloud2::ConstPtr& msg);

  void SendRelocRequest(const ScanPose& scan_pose);

  void PublishMarginalizedScanPose(const ScanPose& scan_pose);

  void PublishScanRegistrationResults(
      const fuse_core::Transaction::SharedPtr& transaction_lm,
      const fuse_core::Transaction::SharedPtr& transaction_gm,
      const ScanPose& scan_pose);

  /** subscribe to lidar data */
  ros::Subscriber subscriber_;

  /** Publishers */
  ros::Publisher results_publisher_; // for global mapper
  ros::Publisher reloc_request_publisher_; // for global mapper
  ros::Publisher registration_publisher_init_;
  ros::Publisher registration_publisher_aligned_lm_;
  ros::Publisher registration_publisher_aligned_gm_;
  int published_registration_results_{0};

  /** callback for lidar data */
  using ThrottledCallback =
      fuse_core::ThrottledMessageCallback<sensor_msgs::PointCloud2>;
  ThrottledCallback throttled_callback_;

  /** Needed for outputing the slam results or saving final clouds or graph
   * updates */
  std::list<ScanPose> active_clouds_;

  /** Get access to the active submap being published by the global mapper if
   * running */
  ActiveSubmap& active_submap_ = ActiveSubmap::GetInstance();

  /** Only needed if using LoamMatcher */
  std::shared_ptr<beam_matching::LoamFeatureExtractor> feature_extractor_{
      nullptr};

  std::unique_ptr<scan_registration::ScanRegistrationBase> scan_registration_;

  fuse_core::UUID device_id_;  //!< The UUID of this device

  /** Used to get initial pose estimates */
  std::unique_ptr<frame_initializers::FrameInitializerBase> frame_initializer_;

  bs_common::ExtrinsicsLookupOnline& extrinsics_ =
      bs_common::ExtrinsicsLookupOnline::GetInstance();

  bs_parameters::models::LidarOdometryParams params_;

  std::vector<beam_filtering::FilterParamsType> input_filter_params_;
  int updates_{0};

  ros::Duration reloc_request_period_;
  ros::Time last_reloc_request_time_{ros::Time(0)};

  /** Params that can only be updated here: */
  bool output_graph_updates_{false};
  std::string graph_updates_path_ =
      "/home/nick/results/beam_slam/graph_updates/";
  bool update_scan_registration_map_on_graph_update_{true};
};

}  // namespace bs_models
