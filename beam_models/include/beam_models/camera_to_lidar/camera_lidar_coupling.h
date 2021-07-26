#pragma once

#include <queue>

#include <fuse_core/async_sensor_model.h>
#include <fuse_core/uuid.h>
#include <fuse_models/common/throttled_callback.h>
#include <fuse_variables/point_3d_landmark.h>
#include <std_msgs/Int64.h>


#include <beam_common/extrinsics_lookup.h>
#include <beam_common/lidar_map.h>
#include <beam_parameters/models/camera_lidar_coupling_params.h>

namespace beam_models {
namespace camera_to_lidar {

class CameraLidarCoupling : public fuse_core::AsyncSensorModel {
 public:
  SMART_PTR_DEFINITIONS(CameraLidarCoupling);

  CameraLidarCoupling();

  ~CameraLidarCoupling() override = default;

 private:
  void onStart() override;

  void onInit() override;

  void onStop() override;

  /**
   * @brief this calls ProcessKeypointsQueue which essentially creates a
   * transaction for constaints on all new keypoints currently in the queue
   * @TODO: Do we want to also update old camera-lidar constraints? Since lidar
   * map may be more updated and same with keypoint positions which may result
   * in better correspondences. For now this is left as a todo
   */
  void onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph_msg) override;

  /**
   * @brief process incoming landmark ids. This just adds them to the queue. The
   * constraint generation happens on graph update
   */
  void process(const std_msgs::Int64::ConstPtr& msg);

  /**
   * @brief Iterate through all keypoints in the queue, find correspondences
   * with the lidar map and generate 3D-3D constraints
   */
  fuse_core::Transaction::SharedPtr ProcessKeypointsQueue(
      fuse_core::Graph::ConstSharedPtr graph_msg);

  Eigen::Vector3d GetLidarCorrespondence(
      const fuse_variables::Point3DLandmark::SharedPtr& landmark);

  /** subscribe to keypoint ids */
  ros::Subscriber subscriber_;

  /** callback for lidar data */
  using ThrottledCallback =
      fuse_models::common::ThrottledCallback<std_msgs::Int64>;
  ThrottledCallback throttled_callback_;
  
  /** The UUID of this device */
  fuse_core::UUID device_id_;

  beam_parameters::models::CameraLidarCouplingParams params_;

  // TODO do we need this?
  beam_common::ExtrinsicsLookup& extrinsics_ =
      beam_common::ExtrinsicsLookup::GetInstance();

  // store reference to local lidar map for finding corresponding points
  beam_common::LidarMap& lidar_map_ = beam_common::LidarMap::GetInstance();

  /** store the new keypoint ids to be processed */
  std::queue<uint64_t> keypoints_queue_;

  /** source to add to constraint */
  std::string source_{"CameraLidarCoupling"};

  /** debugging tools - these must be set here */
  bool output_correspondences_{false};
  std::string output_path_ =
      "/home/nick/results/beam_slam/camera_lidar_coupling/";
};

}  // namespace camera_to_lidar
}  // namespace beam_models
