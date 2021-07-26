#pragma once

#include <queue>

#include <fuse_core/async_sensor_model.h>
#include <fuse_core/uuid.h>
#include <fuse_models/common/throttled_callback.h>
#include <fuse_variables/point_3d_landmark.h>
#include <std_msgs/Int64.h>

#include <beam_common/lidar_map.h>
#include <beam_parameters/models/camera_lidar_coupling_params.h>

namespace beam_models {
namespace camera_to_lidar {

/**
 * @brief This class subscribes to a list of landmark ids representing new
 * landmarks added to the graph. It adds these ids to a queue, and on graph
 * update it takes all landmarks and adds a constraint to the lidar data.
 *
 * @todo Right now, we are adding a 3D to 3D constraint between the landmark 3D
 * location and the closest lidar point. We should extend this to different
 * types of contraints. Two options are suggested:
 *
 *  (1) point-to-plane only: for all landmarks, we generate a constraint from
 * the point the a plane defined by 3 neighbouring points. We will have to
 * somehow ensure that it's a plane. We could use LIO-SAM's approach where they
 * check that the offset in the points (in the direction of the lidar) is less
 * than some threshold. We could also do some sampling approach were we select
 * sets of 3 points nearby and find the closest set of points that we think are
 * a plane.
 *
 *  (2) We could choose between point-to-plane or point-to-line but looking at
 * the nearest points and coming up with some criteria for checking (similar to
 * loam or do an eigenvalue analysis). We could also come up with some way to
 * better describe the line, e.g. fit two planes then find the intersecting
 * line.
 *
 * NOTE: these could be novel contributions
 *
 * @todo currently, if a new landmark isn't in the graph or if we can't find a
 * lidar correspondence, we keep the landmark in the queue. We should implement
 * something that eventually drops it from the queue because it's possible that
 * points won't ever get correspondences (e.g., different sensor FOVs)
 */
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

  bool GetLidarCorrespondence(
      const fuse_variables::Point3DLandmark::SharedPtr& landmark,
      const pcl::KdTreeFLANN<pcl::PointXYZ>& lidar_map_search_tree,
      const PointCloud& lidar_map, Eigen::Vector3d& lidar_point);

  /** subscribe to keypoint ids */
  ros::Subscriber subscriber_;

  /** callback for lidar data */
  using ThrottledCallback =
      fuse_models::common::ThrottledCallback<std_msgs::Int64>;
  ThrottledCallback throttled_callback_;

  /** The UUID of this device */
  fuse_core::UUID device_id_;

  beam_parameters::models::CameraLidarCouplingParams params_;

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
