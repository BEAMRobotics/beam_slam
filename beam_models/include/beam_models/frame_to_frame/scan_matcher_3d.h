#pragma once

#include <unordered_map>

#include <fuse_core/async_sensor_model.h>
#include <fuse_core/uuid.h>
#include <fuse_models/common/throttled_callback.h>

#include <beam_utils/pointclouds.h>

#include <beam_constraints/frame_to_frame/frame_to_frame_transaction_base.h>
#include <beam_models/frame_initializers/frame_initializers.h>
#include <beam_models/frame_to_frame/scan_registration/scan_registration_base.h>
#include <beam_common/scan_pose.h>
#include <beam_common/extrinsics_lookup.h>
#include <beam_parameters/models/scan_matcher_3d_params.h>

namespace beam_models {
namespace frame_to_frame {

class ScanMatcher3D : public fuse_core::AsyncSensorModel {
 public:
  SMART_PTR_DEFINITIONS(ScanMatcher3D);

  ScanMatcher3D();

  ~ScanMatcher3D() override = default;

 protected:
  void onStart() override;

  void onInit() override;

  void onStop() override;

  void onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph_msg) override;

  void process(const sensor_msgs::PointCloud2::ConstPtr& msg);

  beam_constraints::frame_to_frame::Pose3DStampedTransaction
  GenerateTransaction(const sensor_msgs::PointCloud2::ConstPtr& msg);

  /** subscribe to lidar data */
  ros::Subscriber subscriber_;

  /** callback for lidar data */
  using ThrottledCallback = fuse_models::common::ThrottledCallback<sensor_msgs::PointCloud2>;
  ThrottledCallback throttled_callback_;

  /** only needed if you want to output the final clouds or graph updates */
  std::list<beam_common::ScanPose> active_clouds_;

  /** Only needed if using LoamMatcher */
  std::shared_ptr<beam_matching::LoamFeatureExtractor> feature_extractor_{
      nullptr};

  std::unique_ptr<ScanRegistrationBase> scan_registration_;

  fuse_core::UUID device_id_;  //!< The UUID of this device

  /** Used to get initial pose estimates */
  std::unique_ptr<frame_initializers::FrameInitializerBase> frame_initializer_;

  beam_common::ExtrinsicsLookup& extrinsics_ =
      beam_common::ExtrinsicsLookup::GetInstance();

  beam_parameters::models::ScanMatcher3DParams params_;
  bool output_graph_updates_{false};
  int updates_{0};
  std::string graph_updates_path_ =
      "/home/nick/results/beam_slam/graph_updates/";
};

}  // namespace frame_to_frame
}  // namespace beam_models
