#pragma once

#include <opencv2/opencv.hpp>
#include <ros/ros.h>

#include <beam_utils/utils.h>
#include <beam_matching/loam/LoamPointCloud.h>

#include <bs_common/bs_msgs.h>

namespace bs_models {

/**
 * @brief This is a singleton class used to store the current active submap.
 * Below are the following tasks done by this class:
 *
 *  (1) Subscribe to /active_submap topic to load the currently active submap
 *  published by the global mapper via SubmapMsgs
 *
 *  (2) To load the active submap data into a useful form
 *
 *  (3) To provide data access tools for
 *
 * Note that all data herein is stored relative to the local mapper's world
 * frame. When the global mapper performs optimization, or loads a previous
 * global map, it keeps track of or estimates the world coordinate frame of the
 * local mapper before outputting the active submap data. For loading offline
 * maps, it takes the first successful relocalization result from the reloc
 * requests sent by the local mapper, and saves the transform from the offline
 * map world frame to the local mapper's world frame. For the case of the online
 * map data (submaps being built by the local mapper) we store the original
 * submap poses that the local mapper outputs, so even though we may transform
 * those with loop closure, their poses relative to the local mapper's world
 * frame is always returned.
 *
 */
class ActiveSubmap {
 public:
  /**
   * @brief Static Instance getter (singleton)
   * @return reference to the singleton
   */
  static ActiveSubmap& GetInstance();

  /**
   * @brief Delete copy constructor
   */
  ActiveSubmap(const ActiveSubmap& other) = delete;

  /**
   * @brief Delete copy assignment operator
   */
  ActiveSubmap& operator=(const ActiveSubmap& other) = delete;

  /**
   * @brief Updates the data with the new submap message
   * @param message odometry message
   */
  void ActiveSubmapCallback(const bs_common::SubmapMsg::ConstPtr& msg);

  /**
   * @brief Gets a list of visual map points in the camera frame
   * @param T_WORLD_CAMERA current frame pose to transform points into
   * @return list of 3d points
   */
  std::vector<Eigen::Vector3d> GetVisualMapPoints(
      const Eigen::Matrix4d& T_WORLD_CAMERA);

  /**
   * @brief Gets a list of descriptors
   * @return list of descriptors
   */
  const std::vector<cv::Mat>& GetDescriptors();

  /**
   * @brief Gets the lidar submap pointcloud
   * @return point cloud
   */
  PointCloud GetLidarMap();

  /**
   * @brief Gets the loam submap cloud
   * @return loam cloud
   */
  beam_matching::LoamPointCloud GetLoamMap();

  /**
   * @brief Gets the lidar submap as a const pointcloud ptr
   * @return point cloud ptr
   */
  const PointCloudPtr GetLidarMapPtr();

  /**
   * @brief Gets the loam submap cloud as a const ptr
   * @return loam cloud ptr
   */
  const beam_matching::LoamPointCloudPtr GetLoamMapPtr();

  /**
   * @brief Removes a visual map point from the submap
   * @param index of point to remove
   */
  void RemoveVisualMapPoint(size_t index);

 private:
  /**
   * @brief Constructor
   */
  ActiveSubmap();

  PointCloudPtr lidar_map_points_;
  beam_matching::LoamPointCloudPtr loam_cloud_;
  std::vector<Eigen::Vector3d> visual_map_points_;
  std::vector<cv::Mat> descriptors_;
  ros::Subscriber submap_subscriber_;
};

}  // namespace bs_models