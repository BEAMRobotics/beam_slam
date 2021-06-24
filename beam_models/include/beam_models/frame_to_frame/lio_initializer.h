#pragma once

#include <queue>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <beam_parameters/models/lio_initializer_params.h>
#include <beam_models/frame_to_frame/imu_preintegration.h>
#include <beam_common/extrinsics_lookup.h>

#include <fuse_core/async_sensor_model.h>

#include <beam_utils/pointclouds.h>

namespace beam_models {
namespace frame_to_frame {

/**
 * @brief This initializer creates an initial trajectory using imu and lidar
 * data, while also initializing imu params including gravity directiona and
 * biases. It then publishes the results over ROS. The goal of this initializer
 * is to get a decent trajectory estimate and imu params that can be used to
 * begin either VIO or LIO. Either VIO or LIO may further refine the poses and
 * IMU params, but this should give a reliable first estimate with scale.
 *
 * NOTE: this has been implemented as a sensor model for ease of use, however,
 * it does not access the fuse graph in any way since the estimation done in the
 * initializer neesd to be de-coupled from the graph used by the main SLAM.
 *
 * NOTE: coordinate frames: all data is transformed to the imu frame asap (see
 * AddPointcloud function) and worked with in the imu frame.
 */
class LioInitializer : public fuse_core::AsyncSensorModel {
 public:
  SMART_PTR_DEFINITIONS(LioInitializer);

  LioInitializer();

  ~LioInitializer() override = default;

  /**
   * @brief Callback for imu processing, this callback will add imu messages to
   * the buffer
   * @param[in] msg - The imu message to process
   */
  void processIMU(const sensor_msgs::Imu::ConstPtr& msg);

  /**
   * @brief Callback for lidar points processing, this callback has most of the
   * intializer implementation
   * @param[in] msg - The lidar message to process
   */
  void processLidar(const sensor_msgs::PointCloud2::ConstPtr& msg);

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
  void onStop() override;

  /**
   * @brief todo
   */
  void ProcessCurrentKeyframe();

  /**
   * @brief Add the current pointcloud to the current keyframe (or cloud
   * aggregate). This will lookup the relative pose between the current keyframe
   * and the frame associated with this time, using imu preintegration. Then
   * transform the current pointcloud into the keyframe coordinate system.
   * @param cloud pointcloud in the lidar sensor frame
   * @param time timestamp associated with this cloud
   */
  void AddPointcloud(const PointCloud& cloud, const ros::Time& time);

 protected:
  ros::Subscriber imu_subscriber_;
  ros::Subscriber lidar_subscriber_;
  std::unique_ptr<ImuPreintegration> imu_preintegration_;
  fuse_core::Graph::ConstSharedPtr graph_;
  beam_parameters::models::LioInitializerParams params_;
  const beam_common::ExtrinsicsLookup& extrinsics_ =
      beam_common::ExtrinsicsLookup::GetInstance();

  ros::Time keyframe_start_time_{0};
  PointCloud keyframe_cloud_;
  Eigen::Matrix4d T_KEYFRAME_WORLD_{Eigen::Matrix4d::Identity()};
  bool initialization_complete_{false};
};
}  // namespace frame_to_frame
}  // namespace beam_models
