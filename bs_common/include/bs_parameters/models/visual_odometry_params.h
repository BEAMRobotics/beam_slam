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

    // send a trigger to IO to set IMU relative state constraint
    getParam<bool>(nh, "trigger_inertial_odom_constraints",
                   trigger_inertial_odom_constraints,
                   trigger_inertial_odom_constraints);

    // keyframe parallax (rotation adjusted as in vins mono)
    getParam<double>(nh, "keyframe_parallax", keyframe_parallax,
                     keyframe_parallax);

    // read detailed vo params from json
    std::string vo_config;
    getParam<std::string>(nh, "vo_config", vo_config, vo_config);
    readVOParams(vo_config);

    // compute reprojection loss
    double reprojection_loss_a = 5.0 * reprojection_information_weight;
    reprojection_loss =
        std::make_shared<fuse_loss::CauchyLoss>(reprojection_loss_a);

    // compute odom cov from info
    odom_covariance_weight =
        1 / (odom_information_weight * odom_information_weight);
  }

  /**
   * @brief Reads visual odom params from a json file
   *
   * @param[in] vo_config string of the vo config under the config folder
   */
  void readVOParams(const std::string& vo_config) {
    if (vo_config.empty()) {
      ROS_ERROR("Empty VO config path, using default params.");
      return;
    }

    std::string vo_params =
        beam::CombinePaths(bs_common::GetBeamSlamConfigPath(), vo_config);
    nlohmann::json J;

    if (!beam::ReadJson(vo_params, J)) {
      ROS_ERROR("Invalid VO config path, using default params.");
      return;
    }

    try {
      beam::ValidateJsonKeysOrThrow(
          {"max_triangulation_distance", "max_triangulation_reprojection",
           "reprojection_information_weight", "use_idp",
           "track_outlier_pixel_threshold", "required_points_to_refine",
           "use_frame_init_q_to_localize", "use_frame_init_p_to_localize"},
          J);
      use_idp = J["use_idp"];
      max_triangulation_distance = J["max_triangulation_distance"];
      max_triangulation_reprojection = J["max_triangulation_reprojection"];
      reprojection_information_weight = J["reprojection_information_weight"];
      track_outlier_pixel_threshold = J["track_outlier_pixel_threshold"];
      required_points_to_refine = J["required_points_to_refine"];
      use_frame_init_q_to_localize = J["use_frame_init_q_to_localize"];
      use_frame_init_p_to_localize = J["use_frame_init_p_to_localize"];
    } catch (...) { ROS_WARN("Invalid VO config, using default params."); }

    if (use_standalone_vo) {
      try {
        beam::ValidateJsonKeysOrThrow({"standalone_vo_params"}, J);
      } catch (...) {
        ROS_WARN("Standalone VO params empty, using default params.");
        return;
      }
      nlohmann::json standalone_vo_J = J["standalone_vo_params"];
      try {
        beam::ValidateJsonKeysOrThrow({"odom_information_weight",
                                       "marginalization_prior_weight",
                                       "invalid_localization_covariance_weight",
                                       "imu_q_covariance", "imu_p_covariance"},
                                      standalone_vo_J);
        odom_information_weight = standalone_vo_J["odom_information_weight"];
        marginalization_prior_weight =
            standalone_vo_J["marginalization_prior_weight"];
        imu_q_covariance = standalone_vo_J["imu_q_covariance"];
        imu_p_covariance = standalone_vo_J["imu_p_covariance"];
        invalid_localization_covariance_weight =
            standalone_vo_J["invalid_localization_covariance_weight"];
      } catch (...) {
        ROS_WARN("Invalid standalone VO config, using default params.");
      }
    }
  }

  // frame init params
  std::string frame_initializer_config{};
  Eigen::Matrix<double, 6, 6> prior_covariance;
  double prior_information_weight{0};

  // yaml vo params
  bool use_standalone_vo{false};
  bool trigger_inertial_odom_constraints{true};
  double keyframe_parallax{20.0};

  // main vo params
  bool use_idp{false};
  double max_triangulation_distance{30.0};
  double max_triangulation_reprojection{30.0};
  double reprojection_information_weight{1.0};
  double track_outlier_pixel_threshold{1.0};
  int required_points_to_refine{30};
  bool use_frame_init_q_to_localize{true};
  bool use_frame_init_p_to_localize{false};

  // vo params used only when standalone vo is true
  double marginalization_prior_weight{1e-9};
  double odom_information_weight{100.0};
  double odom_covariance_weight{1e-4};
  double imu_q_covariance{1e-1};
  double imu_p_covariance{1e-1};
  double invalid_localization_covariance_weight{1e2};

  fuse_core::Loss::SharedPtr reprojection_loss;
};
}} // namespace bs_parameters::models
