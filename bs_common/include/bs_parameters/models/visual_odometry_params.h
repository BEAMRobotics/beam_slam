#pragma once

#include <bs_parameters/parameter_base.h>
#include <fuse_loss/cauchy_loss.h>
#include <fuse_loss/geman_mcclure_loss.h>
#include <fuse_loss/welsch_loss.h>

#include <ros/node_handle.h>
#include <ros/param.h>

#include <string>
#include <vector>

namespace bs_parameters { namespace models {

/**
 * @brief Defines the set of parameters required by the ScanMatcher class
 */
struct VisualOdometryParams : public ParameterBase {
public:
  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFromROS(const ros::NodeHandle& nh) final {
    /** subscribing topics */
    getParam<std::string>(
        nh, "visual_measurement_topic", visual_measurement_topic,
        "/local_mapper/visual_feature_tracker/visual_measurements");

    // config for frame initializer
    getParamRequired<std::string>(nh, "frame_initializer_config",
                                  frame_initializer_config);

    // frame initializer priorx
    getParam<bool>(nh, "use_pose_priors", use_pose_priors, false);

    // prior on frame init poses if desired
    double prior_covariance_weight;
    getParam<double>(nh, "prior_covariance_weight", prior_covariance_weight, 0.1);
    prior_covariance = prior_covariance_weight * prior_covariance;

    // feature tracker container size
    getParam<int>(nh, "max_container_size", max_container_size, 300);

    // period in which to perform reloc requests
    getParam<double>(nh, "reloc_request_period", reloc_request_period, 1.0);

    // keyframe decision parameters
    getParam<double>(nh, "keyframe_parallax", keyframe_parallax, 40.0);
    getParam<double>(nh, "keyframe_max_duration", keyframe_max_duration, 0.5);
    getParam<double>(nh, "reprojection_covariance_weight",
                     reprojection_covariance_weight, 0.01);

    double reprojection_loss_a;
    getParam<double>(nh, "reprojection_loss_a", reprojection_loss_a, 1.0);
    // reprojection loss
    reprojection_loss =
        std::make_shared<fuse_loss::WelschLoss>(reprojection_loss_a);
  }

  std::string visual_measurement_topic{
      "/local_mapper/visual_feature_tracker/visual_measurements"};

  std::string frame_initializer_config{};
  Eigen::Matrix<double, 6, 6> prior_covariance{
      Eigen::Matrix<double, 6, 6>::Identity()};

  int max_container_size{300};
  bool use_pose_priors{false};
  double reloc_request_period{3.0};
  double keyframe_parallax{40.0};
  double keyframe_max_duration{0.5};
  double reprojection_covariance_weight{0.01};
  fuse_core::Loss::SharedPtr reprojection_loss;
};
}} // namespace bs_parameters::models
