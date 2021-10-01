#pragma once

#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>
#include <ros/param.h>

#include <beam_utils/angles.h>

#include <bs_parameters/parameter_base.h>

namespace bs_parameters {
namespace models {

/**
 * @brief Defines the set of parameters required by the ScanMatcher class
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
    getParamRequired<std::string>(nh, "type", type);

    /** Outputs scans as PCD files IFF not empty */
    getParam<std::string>(nh, "scan_output_directory", scan_output_directory,
                          "");

    /** DEFAULT_PATH uses the appropriate config in
     * beam_slam_launch/config/matcher_config/. Setting to empty will use the
     * default params defined in the class */
    getParam<std::string>(nh, "matcher_params_path", matcher_params_path, "");

    /** DEFAULT_PATH uses the appropriate config in
     * beam_slam_launch/config/registration_config/. Setting to empty will use
     * the default params defined in the class */
    getParam<std::string>(nh, "registration_config_path",
                          registration_config_path, "");

    /** DEFAULT_PATH uses the config in
     * beam_slam_launch_config/config/registration_config/input_filters.json.
     * Setting to empty uses no filters. */
    getParam<std::string>(nh, "input_filters_config_path",
                          input_filters_config_path, "");

    /** Sets all diagonals on the covariance matrix to this value. Can also
     * specify matcher_noise_diagonal instead which takes priority. Default:
     * 1e-9. */
    getParam<double>(nh, "matcher_noise", matcher_noise, 1e-9);

    /** Use this to specify matcher covariance by diagonal instead of by value.
     * This will override mathcer_noise param. */
    nh.param("matcher_noise_diagonal", matcher_noise_diagonal,
             matcher_noise_diagonal);

    /** Options: ODOMETRY, POSEFILE */
    getParam<std::string>(nh, "frame_initializer_type", frame_initializer_type,
                          frame_initializer_type);

    /** for ODOMETRY: topic, for POSEFILE: path */
    getParam<std::string>(nh, "frame_initializer_info", frame_initializer_info,
                          frame_initializer_info);

    /** if not set to zero, a prior will be added to the frame initializer poses
     * with this noise set to the diagonal of the covariance */
    getParam<double>(nh, "frame_initializer_prior_noise",
                     frame_initializer_prior_noise, 0);

    /** Minimum time between each reloc reequest. If set to zero, it will not
     * send any. Relocs are sent each time a scan pose receives its first graph
     * update, if the elapsed time since the last reloc request is greater than
     * this min parameter. We want to make sure the reloc request has a good
     * initial estimate of the scan pose before sending the reloc request
     * because this pose is used to estimate the transform between the offline
     * map and the online map. Therefore we don't want to send the reloc request
     * right when sending the scan pose to the graph. That being said, the reloc
     * request cannot be at a higher frequency than the optimizer. */
    getParam<double>(nh, "reloc_request_period", reloc_request_period, 1);

    /** Optional For Odometry frame initializer */
    getParam<std::string>(nh, "sensor_frame_id_override",
                          sensor_frame_id_override, "");
  }

  std::string input_topic;
  bool output_loam_points{true};
  bool output_lidar_points{true};
  bool publish_active_submap{false};
  bool publish_local_map{false};
  bool publish_registration_results{false};
  std::string frame_initializer_type{"ODOMETRY"};
  std::string frame_initializer_info{""};

  std::vector<double> matcher_noise_diagonal{0, 0, 0, 0, 0, 0};
  double matcher_noise;
  double frame_initializer_prior_noise;
  std::string type;
  double reloc_request_period;
  std::string matcher_params_path;
  std::string registration_config_path;
  std::string input_filters_config_path;
  std::string scan_output_directory;

  // Optional For Odometry frame initializer
  std::string sensor_frame_id_override;
};

}  // namespace models
}  // namespace bs_parameters
