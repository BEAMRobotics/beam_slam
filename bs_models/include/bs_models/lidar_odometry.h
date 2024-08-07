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
#include <beam_utils/time.h>

#include <bs_common/extrinsics_lookup_online.h>
#include <bs_models/frame_initializers/frame_initializer.h>
#include <bs_models/lidar/scan_pose.h>
#include <bs_models/scan_registration/scan_registration_base.h>
#include <bs_parameters/models/lidar_odometry_params.h>

namespace bs_models {

/**
 * @brief todo
 *
 * Initialization: this doesn't start until the first graph update is received.
 * The goal of this is to ensure the slam initializer is always run
 */
class LidarOdometry : public fuse_core::AsyncSensorModel {
public:
  FUSE_SMART_PTR_DEFINITIONS(LidarOdometry);

  LidarOdometry();

  ~LidarOdometry() override = default;

private:
  void onInit() override;

  void onStart() override;

  void onStop() override;

  void onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph_msg) override;

  void process(const sensor_msgs::PointCloud2::ConstPtr& msg);

  void SetupRegistration();

  void PublishMarginalizedScanPose(const std::shared_ptr<ScanPose>& scan_pose);

  void SaveMarginalizedScanPose(const std::shared_ptr<ScanPose>& scan_pose);

  void PublishTfTransform(const Eigen::Matrix4d& T_Child_Parent,
                          const std::string& child_frame,
                          const std::string& parent_frame,
                          const ros::Time& time);

  void PublishExtrinsics(fuse_core::Graph::ConstSharedPtr graph_msg);

  /** subscribe to lidar data */
  ros::Subscriber subscriber_;

  /** Publishers */
  ros::Publisher results_publisher_; // for global mapper
  ros::Publisher reset_publisher_;

  ros::Publisher odom_publisher_;
  int odom_publisher_counter_{0};
  ros::Publisher odom_publisher_marginalized_;
  int odom_publisher_marginalized_counter_{0};
  ros::Publisher imu_constraint_trigger_publisher_;
  int imu_constraint_trigger_counter_{0};

  tf::TransformBroadcaster tf_broadcaster_;

  /** callback for lidar data */
  using ThrottledCallback =
      fuse_core::ThrottledMessageCallback<sensor_msgs::PointCloud2>;
  ThrottledCallback throttled_callback_;
  std::deque<sensor_msgs::PointCloud2::ConstPtr> scan_buffer_;

  /** Needed for outputing the slam results or saving final clouds or graph
   * updates */
  std::list<std::shared_ptr<ScanPose>> active_clouds_;

  /** Only needed if using LoamMatcher */
  std::shared_ptr<beam_matching::LoamFeatureExtractor> feature_extractor_;

  // register scans to map
  std::unique_ptr<scan_registration::ScanRegistrationBase> scan_registration_;

  fuse_core::UUID device_id_; //!< The UUID of this device
  fuse_core::UUID extrinsics_position_uuid_;
  fuse_core::UUID extrinsics_orientation_uuid_;

  /** Used to get initial pose estimates */
  std::unique_ptr<bs_models::FrameInitializer> frame_initializer_;

  bs_common::ExtrinsicsLookupOnline& extrinsics_ =
      bs_common::ExtrinsicsLookupOnline::GetInstance();

  bs_parameters::models::LidarOdometryParams params_;

  std::vector<beam_filtering::FilterParamsType> input_filter_params_;

  int updates_{0};
  Eigen::Matrix4d T_World_BaselinkLast_{Eigen::Matrix4d::Identity()};
  ros::Time last_scan_pose_time_{ros::Time(0)};
  std::string graph_updates_path_;
  std::string marginalized_scans_path_;
  std::string registration_results_path_;
  ros::Time last_map_update_time_{0};
  int skipped_scans_in_a_row_{0};
  bool resetting_{false};

  /** Params that can only be updated here: */
  bool update_registration_map_all_scans_{false};
  bool update_registration_map_in_batch_{false};
  ros::Duration registration_map_batch_update_dur_{10};
  bool use_frame_init_relative_{true};
  int max_scan_buffer_size_{4};
  bool publish_extrinsics_{false};
  bool log_registration_time_{false};

  beam::HighResolutionTimer timer_;
};

} // namespace bs_models
