#pragma once

#include <numeric>

#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>
#include <ros/param.h>

#include <beam_utils/filesystem.h>
#include <beam_utils/pointclouds.h>

#include <bs_common/utils.h>
#include <bs_parameters/parameter_base.h>

namespace bs_parameters { namespace models {

/**
 * @brief Defines the set of parameters required by the LidarOdometry class
 */
struct LidarOdometryParams : public ParameterBase {
public:
  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFromROS(const ros::NodeHandle& nh) final {
    /** Input lidar topic */
    getParamRequired<std::string>(nh, "input_topic", input_topic);

    /** If set to true, it will output the loam points of the marginalized scan
     * poses */
    getParam<bool>(nh, "output_loam_points", output_loam_points,
                   output_loam_points);

    /** If set to true, it will output the lidar points of the marginalized scan
     * poses */
    getParam<bool>(nh, "output_lidar_points", output_lidar_points,
                   output_lidar_points);

    /** If set to true, it will output all points in the active submap */
    getParam<bool>(nh, "publish_active_submap", publish_active_submap,
                   publish_active_submap);

    /** If set to true, it will output all points in the registration map. This
     * is the local map for the lidar */
    getParam<bool>(nh, "publish_local_map", publish_local_map,
                   publish_local_map);

    /** If set to true, it will publish all scan registration results including
     * the initial scan, the aligned to local RegistrationMap and the aligned to
     * ActiveSubmap
     */
    getParam<bool>(nh, "publish_registration_results",
                   publish_registration_results, publish_registration_results);

    getParam<bool>(nh, "save_graph_updates", save_graph_updates,
                   save_graph_updates);

    getParam<bool>(nh, "save_scan_registration_results",
                   save_scan_registration_results,
                   save_scan_registration_results);

    getParam<bool>(nh, "save_marginalized_scans", save_marginalized_scans,
                   save_marginalized_scans);

    /** Outputs scans as PCD files IFF not empty */
    getParam<std::string>(nh, "scan_output_directory", scan_output_directory,
                          scan_output_directory);

    /** Matcher params for local registration. Provide path relative to config
     * folder */
    std::string local_matcher_config_rel;
    getParam<std::string>(nh, "local_matcher_config", local_matcher_config_rel,
                          local_matcher_config_rel);
    if (!local_matcher_config_rel.empty()) {
      local_matcher_config = beam::CombinePaths(
          bs_common::GetBeamSlamConfigPath(), local_matcher_config_rel);
    }

    /** Matcher params for global registration.Provide path relative to config
     * folder */
    std::string global_matcher_config_rel;
    getParam<std::string>(nh, "global_matcher_config",
                          global_matcher_config_rel, global_matcher_config_rel);
    if (!global_matcher_config_rel.empty()) {
      global_matcher_config = beam::CombinePaths(
          bs_common::GetBeamSlamConfigPath(), global_matcher_config_rel);
    }

    /** Scan registration config path for local registration.Provide path
     * relative to config folder  */
    std::string local_registration_config_rel;
    getParam<std::string>(nh, "local_registration_config",
                          local_registration_config_rel,
                          local_registration_config_rel);
    if (!local_registration_config_rel.empty()) {
      local_registration_config = beam::CombinePaths(
          bs_common::GetBeamSlamConfigPath(), local_registration_config_rel);
    }

    /**
     * type of lidar. Options: VELODYNE, OUSTER. This is needed so we know how
     * to convert the PointCloud2 msgs in the lidar odometry.
     */
    std::string lidar_type_str = "VELODYNE";
    getParam<std::string>(nh, "lidar_type", lidar_type_str, lidar_type_str);
    auto iter = LidarTypeStringMap.find(lidar_type_str);
    if (iter == LidarTypeStringMap.end()) {
      ROS_ERROR("Invalid lidar type input param, using default (VELODYNE). "
                "Options: %s",
                GetLidarTypes().c_str());
      lidar_type = LidarType::VELODYNE;
    } else {
      lidar_type = iter->second;
    }

    /** relative file path to input filters config */
    getParam<std::string>(nh, "input_filters_config", input_filters_config,
                          input_filters_config);

    /** Options: TRANSFORM, ODOMETRY, POSEFILE */
    std::string frame_initializer_config_rel;
    getParam<std::string>(nh, "frame_initializer_config",
                          frame_initializer_config_rel,
                          frame_initializer_config_rel);
    if (!frame_initializer_config_rel.empty()) {
      frame_initializer_config = beam::CombinePaths(
          bs_common::GetBeamSlamConfigPath(), frame_initializer_config_rel);
    }

    /** Weighting factor on lidar scan registration measurements.
    This gets applied to the sqrt inv cov such that: E = (w sqrt(cov^-1)) *
    Residuals */
    getParam<double>(nh, "lidar_information_weight", lidar_information_weight,
                     lidar_information_weight);

    /** Minimum time between each reloc request. If set to zero, it will not
     * send any. Relocs are sent each time a scan pose receives its first graph
     * update, if the elapsed time since the last reloc request is greater than
     * this min parameter. We want to make sure the reloc request has a good
     * initial estimate of the scan pose before sending the reloc request
     * because this pose is used to estimate the transform between the offline
     * map and the online map. Therefore we don't want to send the reloc request
     * right when sending the scan pose to the graph. That being said, the reloc
     * request cannot be at a higher frequency than the optimizer. */
    getParam<double>(nh, "reloc_request_period", reloc_request_period,
                     reloc_request_period);

    // Prior weight on frame init poses if desired. If set to 0 then no prior
    // will be added. Since we store covariance here, but we want to specify a
    // weight on the sqrt inv covariance to be consistent with other prior
    // weights, we calculate the equivalent weight according to:
    //   w sqrt(cov^-1) = sqrt[(w' cov)^-1] => w' = 1/w^2
    double prior_information_weight;
    getParam<double>(nh, "prior_information_weight", prior_information_weight,
                     prior_information_weight);
    if (prior_information_weight != 0) {
      double cov_weight =
          1 / (prior_information_weight * prior_information_weight);
      prior_covariance = cov_weight * Eigen::Matrix<double, 6, 6>::Identity();
    }
  }

  // Scan Registration Params
  std::string local_registration_config;
  std::string local_matcher_config;
  std::string global_matcher_config;

  // General params
  std::string input_topic;
  std::string frame_initializer_config{""};
  std::string input_filters_config{""};
  std::string scan_output_directory{""};

  double reloc_request_period;
  double lidar_information_weight{1.0};
  double prior_information_weight{0};

  bool output_loam_points{true};
  bool output_lidar_points{true};
  bool publish_active_submap{false};
  bool publish_local_map{false};
  bool publish_registration_results{false};
  bool save_graph_updates{false};
  bool save_scan_registration_results{false};
  bool save_marginalized_scans{true};

  LidarType lidar_type{LidarType::VELODYNE};

  Eigen::Matrix<double, 6, 6> prior_covariance;
};

}} // namespace bs_parameters::models
