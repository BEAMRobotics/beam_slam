#pragma once

#include <unordered_map>

#include <beam_utils/pointclouds.h>

#include <beam_models/frame_to_frame/frame_to_frame_sensor_model_base.h>
#include <beam_models/frame_to_frame/multi_scan_registration.h>
#include <beam_models/frame_to_frame/scan_pose.h>
#include <beam_parameters/models/scan_matcher_3d_params.h>

namespace beam_models { namespace frame_to_frame {

class ScanMatcher3D
    : public FrameToFrameSensorModelBase<
          sensor_msgs::PointCloud2,
          beam_parameters::models::ScanMatcher3DParams,
          beam_constraints::frame_to_frame::Pose3DStampedTransaction> {
public:
  SMART_PTR_DEFINITIONS(ScanMatcher3D);

  ScanMatcher3D();

  ~ScanMatcher3D() override = default;

beam_constraints::frame_to_frame::Pose3DStampedTransaction
      GenerateTransaction(const sensor_msgs::PointCloud2::ConstPtr& msg);

protected:
  void onInit() override;

  void onStop() override;

  void onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph_msg) override;

  /** only needed if you want to output the final clouds or graph updates */
  std::list<ScanPose> active_clouds_; 

  /** Only needed if using LoamMatcher */
  std::shared_ptr<beam_matching::LoamFeatureExtractor> feature_extractor_{nullptr};

  std::unique_ptr<MultiScanRegistrationBase> multi_scan_registration_;

  // Extra debugging tools: these must be set here, not in the config file
  beam_parameters::models::ScanMatcher3DParams params_;
  bool output_graph_updates_{false};
  int updates_{0};
  std::string graph_updates_path_ =
      "/home/nick/results/beam_slam/graph_updates/";
};

}} // namespace beam_models::frame_to_frame
