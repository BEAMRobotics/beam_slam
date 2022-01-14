#pragma once

#include <fuse_core/async_sensor_model.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>

#include <bs_common/extrinsics_lookup_online.h>
#include <bs_models/frame_initializers/frame_initializers.h>

#include <bs_parameters/models/calibration_params.h>
#include <bs_parameters/models/gt_initializer_params.h>

namespace bs_models {

class GTInitializer : public fuse_core::AsyncSensorModel {
public:
  SMART_PTR_DEFINITIONS(GTInitializer);

  GTInitializer();

  ~GTInitializer() override = default;

  /**
   * @brief Callback for imu processing, this callback has most of the
   * intializer implementation
   * @param[in] msg - The lidar message to process
   */
  void processIMU(const sensor_msgs::Imu::ConstPtr& msg);

  /**
   * @brief Callback for a reset request, which will start the initialization
   * over again
   * @param[in] msg
   */
  void processReset(const std_msgs::Bool::ConstPtr& msg);

protected:
  /**
   * @brief todo
   */
  void onInit() override;

  /**
   * @brief todo
   */
  void onStart() override {}

  /**
   * @brief todo
   */
  void onStop() override {}

  /**
   * @brief publish results of initialization to a InitializedPathMsg
   */
  void PublishResults();

  // parameters
  bs_parameters::models::GTInitializerParams gt_initializer_params_;
  bs_parameters::models::CalibrationParams calibration_params_;

  // subscribers
  ros::Subscriber imu_subscriber_;
  ros::Subscriber reset_subscriber_;
  ros::Publisher results_publisher_;

  // get access to extrinsics singleton
  bs_common::ExtrinsicsLookupOnline& extrinsics_ =
      bs_common::ExtrinsicsLookupOnline::GetInstance();

  // bool for tracking if initialization has completed
  bool initialization_complete_{false};

  std::unique_ptr<frame_initializers::FrameInitializerBase> frame_initializer_;

  std::vector<Eigen::Matrix4d> trajectory_;
  std::vector<ros::Time> times_;
  ros::Time current_pose_time_ = ros::Time(0);
  uint32_t max_poses_;
};
} // namespace bs_models
