#pragma once

#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>
#include <ros/param.h>

#include <beam_utils/angles.h>

#include <beam_parameters/models/frame_to_frame_parameter_base.h>

namespace beam_parameters {
namespace models {

/**
 * @brief Defines the set of parameters required by the ScanMatcher class
 */
struct ScanMatcher3DParams : public FrameToFrameParameterBase {
 public:
  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadExtraParams(const ros::NodeHandle& nh) final {
    getParamRequired<std::string>(nh, "type", type);
    getParam<float>(nh, "downsample_size", downsample_size, 0.03);
    getParam<std::string>(nh, "scan_output_directory", scan_output_directory,
                          "");
    getParam<std::string>(nh, "matcher_params_path", matcher_params_path, "");
    getParam<std::string>(nh, "registration_config_path",
                          registration_config_path, "");
    getParam<double>(nh, "matcher_noise", matcher_noise, 1e-9);

    nh.param("matcher_noise_diagonal", matcher_noise_diagonal,
             matcher_noise_diagonal);

    // get lag_duration from global namespace
    ros::param::get("~lag_duration", lag_duration);
  }

  std::vector<double> matcher_noise_diagonal{0, 0, 0, 0, 0, 0};
  double matcher_noise;
  std::string type;
  float downsample_size;
  double lag_duration;
  std::string matcher_params_path;
  std::string registration_config_path;
  std::string scan_output_directory;
};

}  // namespace models
}  // namespace beam_parameters
