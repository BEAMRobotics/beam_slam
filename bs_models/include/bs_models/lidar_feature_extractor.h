#pragma once

#include <fuse_core/async_sensor_model.h>
#include <fuse_core/fuse_macros.h>
#include <fuse_core/throttled_callback.h>

#include <beam_matching/loam/LoamFeatureExtractor.h>
#include <beam_matching/loam/LoamPointCloud.h>

#include <bs_parameters/models/lidar_feature_extractor_params.h>

namespace bs_models {

class LidarFeatureExtractor : public fuse_core::AsyncSensorModel {
public:
  FUSE_SMART_PTR_DEFINITIONS(LidarFeatureExtractor);

  LidarFeatureExtractor();

  ~LidarFeatureExtractor() override = default;

private:
  void onStart() override;

  void onInit() override;

  void onStop() override;

  void ProcessPointcloud(const sensor_msgs::PointCloud2::ConstPtr& msg);

  void PublishLoamCloud(const beam_matching::LoamPointCloud& cloud,
                        const ros::Time& time, const std::string& frame_id);

  /** subscribe to lidar data */
  ros::Subscriber pointcloud_subscriber_;

  /** Publishers */
  ros::Publisher pub_edges_strong_;
  ros::Publisher pub_edges_weak_;
  ros::Publisher pub_surfaces_strong_;
  ros::Publisher pub_surfaces_weak_;

  /** callbacks */
  using ThrottledCallbackPC =
      fuse_core::ThrottledMessageCallback<sensor_msgs::PointCloud2>;
  ThrottledCallbackPC throttled_callback_pc_;

  bs_parameters::models::LidarFeatureExtractorParams params_;
  int subscriber_queue_size_{5};
  int publisher_queue_size_{10};
  std::unique_ptr<beam_matching::LoamFeatureExtractor> feature_extractor_;
  uint32_t counter_{0};
};

} // namespace bs_models
