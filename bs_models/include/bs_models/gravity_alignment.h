#pragma once

#include <mutex>

#include <fuse_core/async_sensor_model.h>
#include <fuse_core/fuse_macros.h>
#include <fuse_core/throttled_callback.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <bs_common/extrinsics_lookup_online.h>
#include <bs_parameters/models/gravity_alignment_params.h>

namespace bs_models {

class GravityAlignment : public fuse_core::AsyncSensorModel {
public:
  FUSE_SMART_PTR_DEFINITIONS(GravityAlignment);

  /**
   * @brief Default Constructor
   */
  GravityAlignment();

  /**
   * @brief Default Destructor
   */
  ~GravityAlignment() override = default;

private:
  /**
   * @brief Callback for imu processing
   * @param[in] msg - The imu msg to process
   */
  void processIMU(const sensor_msgs::Imu::ConstPtr& msg);

  /**
   * @brief Perform any required initialization for the sensor model
   * (Load parameters from yaml files and read in imu intrinsics)
   * @brief Callback for processing odometry messages, these messages are meant
   * to be poses in which we add constraints to
   * @param[in] msg - The odom msg to process
   */
  void processOdometry(const nav_msgs::Odometry::ConstPtr& msg);

  /**
   * @brief Perform any required initialization for the sensor model
   */
  void onInit() override;

  /**
   * @brief Subscribe to the input topics to start sending transactions to the
   * optimizer
   */
  void onStart() override;

  /**
   * @brief Unsubscribe to the input topics
   */
  void onStop() override {}

  void AddConstraint(const sensor_msgs::Imu::ConstPtr& imu_msg,
                     const ros::Time& stamp);

  void Publish(const sensor_msgs::Imu::ConstPtr& imu_data,
               const nav_msgs::Odometry::ConstPtr& odom_data) const;

  // loadable parameters
  bs_parameters::models::GravityAlignmentParams params_;
  Eigen::Matrix2d covariance_;

  // subscribers
  ros::Subscriber imu_subscriber_;
  ros::Subscriber odom_subscriber_;

  // publishers
  ros::Publisher publisher_;
  int counter_{0};

  // store IMU data up to buffer_duration_ and each time a new odom topic comes
  // in, we create a constraint and clear all IMU data prior to that timestamp
  std::map<int64_t, sensor_msgs::Imu::ConstPtr> imu_buffer_;

  // extrinsics
  bs_common::ExtrinsicsLookupOnline& extrinsics_ =
      bs_common::ExtrinsicsLookupOnline::GetInstance();

  // throttled callbacks for imu
  using ThrottledIMUCallback =
      fuse_core::ThrottledMessageCallback<sensor_msgs::Imu>;
  ThrottledIMUCallback throttled_imu_callback_;

  // throttle callback for odometry
  using ThrottledOdomCallback =
      fuse_core::ThrottledMessageCallback<nav_msgs::Odometry>;
  ThrottledOdomCallback throttled_odom_callback_;
  std::mutex gravity_mutex_;

  // ------------------------------
  // Parameters only tuneable here:
  std::string source_{"GravityAlignment"};

  // 5 second window of IMU data to store
  int64_t buffer_duration_in_ns_{5000000000};

  // max offset between odom msg and closest IMU msg
  int64_t max_time_offset_in_ns_{100000000}; // 0.1s

  bool publish_results_{true};
  double gravity_vector_length_{0.75}; // m
  Eigen::Vector3d g_in_World_{0, 0, -1};
  // ------------------------------
};

} // namespace bs_models
