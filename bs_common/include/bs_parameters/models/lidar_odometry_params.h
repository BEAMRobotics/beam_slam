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

    getParam<bool>(nh, "save_scan_registration_results", save_scan_registration_results,
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

    /** Minimum time between each reloc reequest. If set to zero, it will not
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

    /** Use this to specify prior covariance by diagonal. If all diagonal
     * elements are set to zero, priors will not be added */
    std::vector<double> prior_diagonal;
    nh.param("frame_initializer_prior_noise_diagonal", prior_diagonal,
             prior_diagonal);
    if (prior_diagonal.size() != 6) {
      ROS_ERROR("Invalid gm_noise_diagonal params, required 6 params, "
                "given: %zu. Using default (0.1 for all)",
                prior_diagonal.size());
      prior_diagonal = std::vector<double>{0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
    }
    if (std::accumulate(prior_diagonal.begin(), prior_diagonal.end(), 0.0) ==
        0.0) {
      ROS_INFO("Prior diagonal set to zero, not adding priors");
      use_pose_priors = false;
    }
    for (int i = 0; i < 6; i++) { prior_covariance(i, i) = prior_diagonal[i]; }
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

  bool output_loam_points{true};
  bool output_lidar_points{true};
  bool publish_active_submap{false};
  bool publish_local_map{false};
  bool publish_registration_results{false};
  bool use_pose_priors{true};
  bool save_graph_updates{false};
  bool save_scan_registration_results{false};
  bool save_marginalized_scans{true};

  LidarType lidar_type{LidarType::VELODYNE};

  Eigen::Matrix<double, 6, 6> prior_covariance{
      Eigen::Matrix<double, 6, 6>::Identity()};
};

}} // namespace bs_parameters::models
