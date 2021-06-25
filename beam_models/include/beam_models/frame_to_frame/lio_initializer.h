#pragma once

#include <list>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <fuse_core/async_sensor_model.h>
#include <fuse_graphs/hash_graph.h>

#include <beam_utils/pointclouds.h>
#include <beam_matching/Matchers.h>
#include <beam_matching/loam/LoamPointCloud.h>
#include <beam_matching/loam/LoamFeatureExtractor.h>

#include <beam_parameters/models/lio_initializer_params.h>
#include <beam_models/frame_to_frame/imu_preintegration.h>
#include <beam_models/frame_to_frame/scan_registration/scan_to_map_registration.h>
#include <beam_common/extrinsics_lookup.h>
#include <beam_common/scan_pose.h>

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
   * @brief Register scan against previous and then add to the queue of scan
   * poses. After processing the keyframe, it will check if we are ready to
   * start optimizing. To start optimizing, we check that the time window is
   * greater than the time_window param, and that the trajectory length is
   * greater than min_trajectory_distance. If the time window is larger than the
   * min, but the trajectory is not long enough, throw away the first scan in
   * the queue and continue collecting keyframes
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
  void AddPointcloudToKeyframe(const PointCloud& cloud, const ros::Time& time);

  /**
   * @brief iterates through all keypoints in the list and add up the change in
   * position between each keyframe.
   * @return trajectory length
   */
  double CalculateTrajectoryLength();

  /**
   * @brief match two scan poses and get resulting transform between them
   * @param scan_pose_1
   * @param scan_pose_2
   * @param T_CLOUD1_CLOUD2 reference to result (transform from cloud 2 to cloud
   * 1)
   * @return true if match was successful
   */
  bool MatchScans(const beam_common::ScanPose& scan_pose_1,
                  const beam_common::ScanPose& scan_pose_2,
                  Eigen::Matrix4d& T_CLOUD1_CLOUD2);

  /**
   * @brief checked the transform between two scans (from scan registration
   * results) against the max motion params to determine if the scan
   * registration was a success of not. This will only eliminate very bad scan
   * registration results.
   * @param T_measured pose refinement between two scans that have been already transformed into a common reference frame
   * @return true if rotation and translation are below threshold set in params
   */
  bool PassedRegThreshold(const Eigen::Matrix4d& T_measured);

  /**
   * @brief create factor graph of scan registration and imu measurements, then
   * solve and publish results.
   */
  void Optimize();

  /**
   * @brief Save three types of scans to separate folders: 
   * 
   * (1) scans transformed to world frame using imu pre-integration pose estimate
   * (2) scans transformed to world frame using loam refined pose estimates
   * (3) scans transformed to world frame using final pose estimates from factor graph
   * 
   */
  void OutputResults();

 protected:
  // subscribers
  ros::Subscriber imu_subscriber_;
  ros::Subscriber lidar_subscriber_;

  // imu data handler
  std::unique_ptr<ImuPreintegration> imu_preintegration_;

  // main parameters
  beam_parameters::models::LioInitializerParams params_;

  // get access to extrinsics singleton
  beam_common::ExtrinsicsLookup& extrinsics_ =
      beam_common::ExtrinsicsLookup::GetInstance();

  // scan registration objects
  std::unique_ptr<ScanToMapLoamRegistration> scan_registration_;
  std::unique_ptr<beam_matching::Matcher<LoamPointCloudPtr>> matcher_;
  std::shared_ptr<beam_matching::LoamFeatureExtractor> feature_extractor_;

  // store all current keyframes to be processed
  std::list<beam_common::ScanPose> keyframes_;

  // keep track of the current keyframe
  ros::Time keyframe_start_time_{0};
  PointCloud keyframe_cloud_;
  Eigen::Matrix4d T_WORLD_KEYFRAME_{Eigen::Matrix4d::Identity()};

  // bool for tracking if initialization has completed
  bool initialization_complete_{false};

  // optimizer
  fuse_core::Graph::SharedPtr graph_;
};
}  // namespace frame_to_frame
}  // namespace beam_models
