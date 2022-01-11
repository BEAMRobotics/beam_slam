#pragma once

#include <queue>

#include <bs_common/bs_msgs.h>
#include <fuse_core/async_sensor_model.h>
#include <fuse_core/throttled_callback.h>
#include <sensor_msgs/Imu.h>

#include <bs_common/extrinsics_lookup_online.h>
#include <bs_models/imu_preintegration.h>
#include <bs_parameters/models/calibration_params.h>
#include <bs_parameters/models/inertial_odometry_params.h>

namespace bs_models {

using namespace bs_common;

class InertialOdometry : public fuse_core::AsyncSensorModel {
public:
  SMART_PTR_DEFINITIONS(InertialOdometry);

  /**
   * @brief Default Constructor
   */
  InertialOdometry();

  /**
   * @brief Default Destructor
   */
  ~InertialOdometry() override = default;

private:
  /**
   * @brief Callback for imu processing, this will make sure the imu messages
   * are added to the buffer at the correct time
   * @param[in] msg - The imu msg to process
   */
  void processIMU(const sensor_msgs::Imu::ConstPtr &msg);

  /**
   * @brief Callback for path processing, this path is provided by LIO for
   * initialization
   * @param[in] msg - The path to process
   */
  void processInitPath(const InitializedPathMsg::ConstPtr &msg);

  /**
   * @brief Perform any required initialization for the sensor model
   * (Load parameters from yaml files and read in imu intrinsics)
   */
  void onInit() override;

  /**
   * @brief Subscribe to the input topics to start sending transactions to the
   * optimizer
   */
  void onStart() override;

  /**
   * @brief Unsubscribe to the input topic
   */
  void onStop() override;

  /**
   * @brief Callback for when a newly optimized graph is available
   */
  void onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph_msg) override;

  /**
   * @brief Process an imu message by adding it to the preintegrator, and if
   * designated time has elapsed since previous state then register a
   * transaction and send to the graph
   */
  void RegisterImuMessage(const sensor_msgs::Imu &msg);

  fuse_core::UUID device_id_; //!< The UUID of this device
  // loadable camera parameters
  bs_parameters::models::InertialOdometryParams inertial_params_;

  // calibration parameters
  bs_parameters::models::CalibrationParams calibration_params_;

  // subscribers
  ros::Subscriber imu_subscriber_;
  ros::Subscriber path_subscriber_;

  // publishers
  ros::Publisher init_odom_publisher_;
  ros::Publisher inertial_pose_stamps_publisher_;

  // buffer imu messages when initializing
  std::queue<sensor_msgs::Imu> imu_buffer_;

  // callbacks for messages
  using ThrottledIMUCallback =
      fuse_core::ThrottledMessageCallback<sensor_msgs::Imu>;
  ThrottledIMUCallback throttled_imu_callback_;

  // imu preintegration object
  std::shared_ptr<ImuPreintegration> imu_preint_;
  bs_models::ImuPreintegration::Params imu_params_;
  Eigen::Vector3d gravity_, bg_, ba_;
  std::vector<Eigen::Vector3d> velocities_;
  double scale_;

  // time of previous imu state
  ros::Time previous_state;

  // robot extrinsics
  bs_common::ExtrinsicsLookupOnline &extrinsics_ =
      bs_common::ExtrinsicsLookupOnline::GetInstance();
};

} // namespace bs_models
