#pragma once

#include <string>
#include <vector>

#include <fuse_loss/cauchy_loss.h>
#include <ros/node_handle.h>
#include <ros/param.h>

#include <bs_common/utils.h>
#include <bs_parameters/parameter_base.h>

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
    // config for frame initializer
    std::string frame_initializer_config_rel;
    getParamRequired<std::string>(nh, "frame_initializer_config",
                                  frame_initializer_config_rel);
    if (!frame_initializer_config_rel.empty()) {
      frame_initializer_config = beam::CombinePaths(
          bs_common::GetBeamSlamConfigPath(), frame_initializer_config_rel);
    }

    // Prior weight on frame init poses if desired. If set to 0 then no prior
    // will be added. Since we store covariance here, but we want to specify a
    // weight on the sqrt inv covariance to be consistent with other prior
    // weights, we calculate the equivalent weight according to:
    //   w sqrt(cov^-1) = sqrt[(w' cov)^-1] => w' = 1/w^2
    getParam<double>(nh, "prior_information_weight", prior_information_weight,
                     prior_information_weight);
    if (prior_information_weight != 0) {
      double cov_weight =
          1 / (prior_information_weight * prior_information_weight);
      prior_covariance = cov_weight * Eigen::Matrix<double, 6, 6>::Identity();
    }

    // whether to add relative constraints only to the main graph
    getParam<bool>(nh, "use_standalone_vo", use_standalone_vo,
                   use_standalone_vo);

    if (use_standalone_vo) {
      getParam<double>(nh, "odom_information_weight", odom_information_weight,
                       odom_information_weight);
      odom_covariance_weight =
          1 / (odom_information_weight * odom_information_weight);
    }

    // send a trigger to IO to set IMU relative state constraint
    getParam<bool>(nh, "trigger_inertial_odom_constraints",
                   trigger_inertial_odom_constraints,
                   trigger_inertial_odom_constraints);

    // keyframe parallax (rotation adjusted as in vins mono)
    getParam<double>(nh, "keyframe_parallax", keyframe_parallax,
                     keyframe_parallax);

    double reprojection_loss_a = 5.0 * reprojection_information_weight;
    // reprojection loss
    reprojection_loss =
        std::make_shared<fuse_loss::CauchyLoss>(reprojection_loss_a);

    // read vo params
    std::string vo_config;
    getParam<std::string>(nh, "vo_config", vo_config, vo_config);
    std::string vo_params =
        beam::CombinePaths(bs_common::GetBeamSlamConfigPath(), vo_config);
    nlohmann::json J;
    if (!beam::ReadJson(vo_params, J)) {
      ROS_ERROR("Cannot read input VO Params, using default.");
    } else {
      try {
        max_triangulation_distance = J["max_triangulation_distance"];
      } catch (...) {
        ROS_ERROR("Missing 'max_triangulation_distance' param in vo config "
                  "file. Using default: 30.0.");
      }

      try {
        max_triangulation_reprojection = J["max_triangulation_reprojection"];
      } catch (...) {
        ROS_ERROR("Missing 'max_triangulation_reprojection' param in vo config "
                  "file. Using default: 80.0.");
      }

      try {
        reprojection_information_weight = J["reprojection_information_weight"];
      } catch (...) {
        ROS_ERROR(
            "Missing 'reprojection_information_weight' param in vo config "
            "file. Using default: 1.0.");
      }

      try {
        use_idp = J["use_idp"];
      } catch (...) {
        ROS_ERROR(
            "Missing 'use_idp' param in vo config file. Using default: False.");
      }
    }
  }

  std::string frame_initializer_config{};
  Eigen::Matrix<double, 6, 6> prior_covariance;

  bool use_standalone_vo{false};
  bool trigger_inertial_odom_constraints{true};
  bool use_idp{false};
  double keyframe_parallax{20.0};
  double reprojection_information_weight{1.0};
  double max_triangulation_distance{30.0};
  double max_triangulation_reprojection{80.0};
  double prior_information_weight{0};
  double odom_information_weight{1.0};
  double odom_covariance_weight;
  fuse_core::Loss::SharedPtr reprojection_loss;
};
}} // namespace bs_parameters::models
