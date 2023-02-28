#pragma once

#include <opencv2/opencv.hpp>
#include <ros/ros.h>

#include <beam_matching/loam/LoamPointCloud.h>

#include <bs_common/bs_msgs.h>
#include <bs_common/extrinsics_lookup_online.h>

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
   * @brief Set the publish_updates_ param. If this is set to true, it will
   * publish the maps as PointCloud2 messages each time it receives an updated
   * SubmapMsg to each of the following topics:
   *
   * - /active_submap/visual_map
   * - /active_submap/lidar_map
   * - /active_submap/loam_map
   *
   * @param publish_updates
   */
  void SetPublishUpdates(bool publish_updates);

  /**
   * @brief Gets a visual map points. If a transform from camera to world is
   * given, it will transform the points to the camera frame, otherwise it will
   * return them in the world frame.
   * @param T_WORLD_CAMERA optional pose of the current camera frame w.r.t the
   * world frame
   * @return vector of 3d points (Eigen Vector3d)
   */
  std::vector<Eigen::Vector3d> GetVisualMapVectorInCameraFrame(
      const Eigen::Matrix4d& T_WORLD_CAMERA =
          Eigen::Matrix4d::Identity()) const;

  /**
   * @brief Gets a visual map points. If a transform from camera to world is
   * given, it will transform the points to the camera frame, otherwise it will
   * return them in the world frame.
   * @param T_WORLD_CAMERA optional pose of the current camera frame w.r.t the
   * world frame
   * @return pointcloud
   */
  PointCloud GetVisualMapCloudInCameraFrame(
      const Eigen::Matrix4d& T_WORLD_CAMERA =
          Eigen::Matrix4d::Identity()) const;

  /**
   * @brief Get a const pointer to the visual map points in the world frame
   * @return pointcloud ptr
   */
  const PointCloudPtr GetVisualMapPoints() const;

  /**
   * @brief Gets a list of descriptors
   * @return list of descriptors
   */
  const std::vector<cv::Mat>& GetDescriptors() const;

  /**
   * @brief Gets the lidar submap as a const pointcloud ptr
   * @return pointcloud ptr
   */
  const PointCloudPtr GetLidarMap() const;

  /**
   * @brief Gets the loam submap cloud as a const ptr
   * @return loam cloud ptr
   */
  const beam_matching::LoamPointCloudPtr GetLoamMapPtr() const;

  /**
   * @brief Removes a visual map point from the submap
   * @param index of point to remove
   */
  void RemoveVisualMapPoint(size_t index);

 private:
  /**
   * @brief Private constructor
   */
  ActiveSubmap();

  /**
   * @brief Publishes map updates. See description of function
   * SetPublishUpdates()
   */
  void Publish() const;

  // data
  PointCloudPtr lidar_map_points_;
  beam_matching::LoamPointCloudPtr loam_cloud_;
  PointCloudPtr visual_map_points_;
  std::vector<cv::Mat> visual_words_;
  std::vector<uint32_t> word_ids_;
  ros::Subscriber submap_subscriber_;
  bs_common::ExtrinsicsLookupOnline& extrinsics_online_ =
      bs_common::ExtrinsicsLookupOnline::GetInstance();

  // publishing map updates:
  bool publish_updates_{false};
  int updates_counter_{0};
  ros::Time update_time_{0};
  ros::Publisher visual_map_publisher_;
  ros::Publisher lidar_map_publisher_;
  ros::Publisher loam_map_publisher_;
};

}  // namespace bs_models