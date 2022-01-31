#pragma once

#include <bs_models/slam_initializers/slam_initializer_base.h>
#include <fuse_graphs/hash_graph.h>
#include <sensor_msgs/Image.h>

#include <beam_calibration/CameraModel.h>
#include <beam_cv/trackers/Trackers.h>

#include <bs_common/bs_msgs.h>
#include <bs_common/extrinsics_lookup_online.h>
#include <bs_models/vision/visual_map.h>
#include <bs_parameters/models/calibration_params.h>
#include <bs_parameters/models/vo_initializer_params.h>

namespace bs_models {

class VOInitializer : public SLAMInitializerBase {
public:
  SMART_PTR_DEFINITIONS(VOInitializer);

  VOInitializer() : SLAMInitializerBase() {}

  ~VOInitializer() override = default;

  /**
   * @brief Callback for image processing, this callback will add visual
   * constraints and triangulate new landmarks when required
   * @param[in] msg - The image to process
   */
  void processImage(const sensor_msgs::Image::ConstPtr& msg);

protected:
  /**
   * @brief todo
   */
  void onInit() override;

  // parameters
  bs_parameters::models::VOInitializerParams vo_initializer_params_;
  bs_parameters::models::CalibrationParams calibration_params_;

  // subscribers
  ros::Subscriber image_subscriber_;
  Eigen::Matrix4d T_cam_baselink_;

  std::deque<ros::Time> kf_times_;

  // computer vision objects
  std::shared_ptr<beam_calibration::CameraModel> cam_model_;
  std::shared_ptr<beam_cv::Tracker> tracker_;

  // optimization stuff
  std::shared_ptr<vision::VisualMap> visual_map_;
  fuse_core::Graph::SharedPtr local_graph_;
};
} // namespace bs_models
