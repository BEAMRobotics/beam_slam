#pragma once

#include <unordered_map>

#include <fuse_core/async_sensor_model.h>
#include <fuse_core/uuid.h>
#include <fuse_models/common/throttled_callback.h>

#include <beam_utils/pointclouds.h>

#include <beam_models/frame_initializers/frame_initializer_base.h>
#include <beam_models/frame_to_frame/scan_pose.h>
#include <beam_models/frame_to_frame/multi_scan_registration.h>
#include <beam_parameters/models/scan_matcher_3d_params.h>

namespace beam_models { namespace frame_to_frame {

class ScanMatcher3D : public fuse_core::AsyncSensorModel {
  using Pose3D = fuse_variables::Position3DStamped;

public:
  SMART_PTR_DEFINITIONS(ScanMatcher3D);
  using ParameterType = beam_parameters::models::ScanMatcher3DParams;

  ScanMatcher3D();

  ~ScanMatcher3D() override = default;

  /**
   * @brief Callback for pose messages
   * @param[in] msg - The pose message to process
   */
  void process(const sensor_msgs::PointCloud2::ConstPtr& msg);

protected:
  /**
   * @brief Perform any required initialization for the sensor model
   *
   * This could include things like reading from the parameter server or
   * subscribing to topics. The class's node handles will be properly
   * initialized before SensorModel::onInit() is called. Spinning of the
   * callback queue will not begin until after the call to SensorModel::onInit()
   * completes.
   */
  void onInit() override;

  /**
   * @brief Subscribe to the input topic to start sending transactions to the
   * optimizer
   */
  void onStart() override;

  /**
   * @brief Unsubscribe from the input topic to stop sending transactions to the
   * optimizer
   */
  void onStop() override;

  void onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph_msg) override;

  fuse_core::UUID device_id_; //!< The UUID of this device
  ParameterType params_;
  ros::Subscriber pointcloud_subscriber_;
  std::unordered_map<std::string, std::shared_ptr<ScanPose>>
      active_clouds_; // only needed if you want to output the final clouds
  std::unique_ptr<frame_initializers::FrameInitializerBase> frame_initializer_;
  std::unique_ptr<MultiScanRegistration> multi_scan_registration_;

  using PointCloudThrottledCallback =
      fuse_models::common::ThrottledCallback<sensor_msgs::PointCloud2>;
  PointCloudThrottledCallback throttled_callback_;
};

}} // namespace beam_models::frame_to_frame
