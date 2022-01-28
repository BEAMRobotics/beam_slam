#pragma once

#include <numeric>

#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>
#include <ros/param.h>

#include <beam_utils/pointclouds.h>

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

    /**The first part is either MULTI (for multi scan registration) or MAP (for
     * scan to map registration), and the second part is the matcher type.
     * Options: MULTIICP, MULTINDT, MULTIGICP, MULTILOAM, MAPLOAM (
     * TODO:  LIBPOINTMATCHER, TEASER */
    getParamRequired<std::string>(nh, "local_registration_type",
                                  local_registration_type);

    /** Matcher type for global registration. Options: ICP, NDT, GICP, LOAM */
    getParamRequired<std::string>(nh, "global_registration_type",
                                  global_registration_type);

    /** Outputs scans as PCD files IFF not empty */
    getParam<std::string>(nh, "scan_output_directory", scan_output_directory,
                          scan_output_directory);

    /** Matcher params for local registration. DEFAULT_PATH uses the
     * appropriate config in beam_slam_launch/config/matcher_config/. Setting to
     * empty will use the default params defined in the class */
    getParam<std::string>(nh, "local_matcher_params_path",
                          local_matcher_params_path, local_matcher_params_path);

    /** Matcher params for global registration. DEFAULT_PATH uses the
     * appropriate config in beam_slam_launch/config/matcher_config/. Setting to
     * empty will use the default params defined in the class */
    getParam<std::string>(nh, "global_matcher_params_path",
                          global_matcher_params_path,
                          global_matcher_params_path);

    /** Scan registration config path for local registration. DEFAULT_PATH uses
     * the appropriate config in beam_slam_launch/config/registration_config/.
     * Setting to empty will use the default params defined in the class */
    getParam<std::string>(nh, "local_registration_config_path",
                          local_registration_config_path,
                          local_registration_config_path);

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

    /** DEFAULT_PATH uses the config in
     * beam_slam_launch_config/config/registration_config/input_filters.json.
     * Setting to empty uses no filters. */
    getParam<std::string>(nh, "input_filters_config_path",
                          input_filters_config_path, input_filters_config_path);

    /** Options: TRANSFORM, ODOMETRY, POSEFILE */
    getParam<std::string>(nh, "frame_initializer_type", frame_initializer_type,
                          frame_initializer_type);

    /** for TRANSFORM: topic, for ODOMETRY: topic, for POSEFILE: path */
    getParam<std::string>(nh, "frame_initializer_info", frame_initializer_info,
                          frame_initializer_info);

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

    /** Optional For Odometry frame initializer */
    getParam<std::string>(nh, "sensor_frame_id_override",
                          sensor_frame_id_override, sensor_frame_id_override);

    /** Use this to specify local mapper covariance by diagonal. If all diagonal
     * elements are set to zero, global map registration will not be performed
     */
    std::vector<double> lm_noise_diagonal;
    nh.param("local_registration_noise_diagonal", lm_noise_diagonal,
             lm_noise_diagonal);
    if (lm_noise_diagonal.size() != 6) {
      ROS_ERROR(
          "Invalid local_mapper_noise_diagonal params, required 6 params, "
          "given: %d. Using default (0.1 for all)",
          lm_noise_diagonal.size());
      lm_noise_diagonal = std::vector<double>{0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
    }
    if (std::accumulate(lm_noise_diagonal.begin(), lm_noise_diagonal.end(),
                        0.0) == 0.0) {
      ROS_INFO("Local mapper noise diagonal set to zero, not performing "
               "registration to local map.");
      register_to_lm = false;
    }
    for (int i = 0; i < 6; i++) {
      local_registration_covariance(i, i) = lm_noise_diagonal[i];
    }

    /** Use this to specify global mapper covariance by diagonal. If all
     * diagonal elements are set to zero, global map registration will not be
     * performed */
    std::vector<double> gm_noise_diagonal;
    nh.param("global_registration_noise_diagonal", gm_noise_diagonal,
             gm_noise_diagonal);
    if (gm_noise_diagonal.size() != 6) {
      ROS_ERROR("Invalid global_registration_noise_diagonal params, required 6 "
                "params, "
                "given: %d. Using default (0.1 for all)",
                gm_noise_diagonal.size());
      gm_noise_diagonal = std::vector<double>{0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
    }
    if (std::accumulate(gm_noise_diagonal.begin(), gm_noise_diagonal.end(),
                        0.0) == 0) {
      ROS_INFO("Global mapper noise diagonal set to zero, not performing "
               "registration to global map.");
      register_to_gm = false;
    }
    for (int i = 0; i < 6; i++) {
      global_registration_covariance(i, i) = gm_noise_diagonal[i];
    }

    /** Use this to specify prior covariance by diagonal. If all diagonal
     * elements are set to zero, priors will not be added */
    std::vector<double> prior_diagonal;
    nh.param("frame_initializer_prior_noise_diagonal", prior_diagonal,
             prior_diagonal);
    if (prior_diagonal.size() != 6) {
      ROS_ERROR("Invalid gm_noise_diagonal params, required 6 params, "
                "given: %d. Using default (0.1 for all)",
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

  // Local Scan Registration Params
  std::string local_registration_type;
  std::string local_registration_config_path{"DEFAULT_PATH"};
  std::string local_matcher_params_path{"DEFAULT_PATH"};
  Eigen::Matrix<double, 6, 6> local_registration_covariance{
      Eigen::Matrix<double, 6, 6>::Identity()};
  bool register_to_lm{true};

  // Global Scan Registration Params
  std::string global_registration_type;
  std::string global_matcher_params_path{"DEFAULT_PATH"};
  Eigen::Matrix<double, 6, 6> global_registration_covariance{
      Eigen::Matrix<double, 6, 6>::Identity()};
  bool register_to_gm{true};

  // General params
  std::string input_topic;
  std::string frame_initializer_type{"ODOMETRY"};
  std::string frame_initializer_info{""};
  std::string input_filters_config_path{""};
  std::string scan_output_directory{""};

  double reloc_request_period;

  bool output_loam_points{true};
  bool output_lidar_points{true};
  bool publish_active_submap{false};
  bool publish_local_map{false};
  bool publish_registration_results{false};
  bool use_pose_priors{true};

  LidarType lidar_type{LidarType::VELODYNE};

  Eigen::Matrix<double, 6, 6> prior_covariance{
      Eigen::Matrix<double, 6, 6>::Identity()};

  // Optional For Odometry frame initializer
  std::string sensor_frame_id_override;
};

}} // namespace bs_parameters::models
