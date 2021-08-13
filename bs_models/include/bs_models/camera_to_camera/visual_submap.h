#pragma once

#include <opencv2/opencv.hpp>

// libbeam
#include <beam_utils/utils.h>
#include <bs_models/SubmapMsg.h>

namespace bs_models { namespace camera_to_camera {

class VisualSubmap {
public:
  /**
   * @brief Default Constructor
   */
  VisualSubmap();

  /**
   * @brief Set the submap with a enw submap message
   * @param msg submap message to update with
   */
  void SetSubmap(const SubmapMsg::ConstPtr& msg);

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
   * @brief Removes a visual map point from the submap
   * @param index of point to remove
   */
  void RemoveVisualMapPoint(size_t index);

protected:
  std::vector<Eigen::Vector3d> visual_map_points_;
  std::vector<cv::Mat> descriptors_;
};

}} // namespace bs_models::camera_to_camera
