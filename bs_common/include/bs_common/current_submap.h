#pragma once

#include <beam_utils/utils.h>
#include <bs_common/SubmapMsg.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

namespace bs_common {

/**
 * @brief This class is used to access the current submap provided by the global
 * mapper
 *
 */
class CurrentSubmap {
public:
  /**
   * @brief Static Instance getter (singleton)
   * @return reference to the singleton
   */
  static CurrentSubmap& GetInstance();

  /**
   * @brief Delete copy constructor
   */
  CurrentSubmap(const CurrentSubmap& other) = delete;

  /**
   * @brief Delete copy assignment operator
   */
  CurrentSubmap& operator=(const CurrentSubmap& other) = delete;

  /**
   * @brief Updates the data with the new submap message
   * @param message odometry message
   */
  void CurrentSubmapCallback(const bs_common::SubmapMsg::ConstPtr& msg);

  /**
   * @brief Gets a list of visual map points in the camera frame
   * @param T_WORLD_CAMERA current frame pose to transform points into
   * @return list of 3d points
   */
  std::vector<Eigen::Vector3d>
      GetVisualMapPoints(const Eigen::Matrix4d& T_WORLD_CAMERA);

  /**
   * @brief Gets a list of descriptors
   * @return list of descriptors
   */
  const std::vector<cv::Mat>& GetDescriptors();

  /**
   * @brief Gets the submaps point cloud
   * @return point cloud
   */
  const pcl::PointCloud<pcl::PointXYZ> GetPointCloud();

  /**
   * @brief Removes a visual map point from the submap
   * @param index of point to remove
   */
  void RemoveVisualMapPoint(size_t index);

private:
  /**
   * @brief Constructor
   */
  CurrentSubmap();

  pcl::PointCloud<pcl::PointXYZ> point_cloud_;
  std::vector<Eigen::Vector3d> visual_map_points_;
  std::vector<cv::Mat> descriptors_;

  ros::Subscriber submap_subscriber_;
};

} // namespace bs_common