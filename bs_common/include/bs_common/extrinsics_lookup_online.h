#pragma once

#include <bs_common/extrinsics_lookup_base.h>
#include <bs_parameters/models/calibration_params.h>

#include <Eigen/Dense>
#include <tf/transform_listener.h>

namespace bs_common {

/**
 * @brief this class can be used to lookup static or dynamic extrinsics
 * calibrations which are being published to tf or tf_static. The 3 types of
 * frames need to be supplied to this class. The prefered way to use this with
 * beam_slam is to add global variables to the config yaml for each of the
 * frames, then each of the sensor models can get the instance of this class
 * with those same global params. See global_mapper.cpp for an example use case.
 */
class ExtrinsicsLookupOnline {
public:
  /**
   * @brief Static Instance getter (singleton)
   * @return reference to the singleton
   */
  static ExtrinsicsLookupOnline& GetInstance();

  /**
   * @brief Delete copy constructor
   */
  ExtrinsicsLookupOnline(const ExtrinsicsLookupOnline& other) = delete;

  /**
   * @brief Delete copy assignment operator
   */
  ExtrinsicsLookupOnline&
      operator=(const ExtrinsicsLookupOnline& other) = delete;

  /**
   * @brief Get a copy of the instrinsics stored herein
   * @param extrinsics copy of the extrinsics_
   */
  ExtrinsicsLookupBase GetExtrinsicsCopy();

  /**
   * @brief See definition in ExtrinsicsLookupBase
   * @param save_filename full path to filename
   */
  void SaveExtrinsicsToJson(const std::string& save_filename);

  /**
   * @brief See definition in ExtrinsicsLookupBase
   * @param save_filename full path to filename
   */
  void SaveFrameIdsToJson(const std::string& save_filename);

  /**
   * @brief get transform from any two frames
   * @param T reference to result
   * @param to_frame 'to frame' of transformation
   * @param from_frame 'from frame' of transformation
   * @param time extrinsics time if extrinsics are not static
   * @return true if lookup was successful
   */
  bool GetTransform(Eigen::Matrix4d& T, const std::string& to_frame,
                    const std::string& from_frame,
                    const ros::Time& time = ros::Time(0));

  /**
   * @brief Gets the extrinsics between camera and IMU
   * @param T reference to result
   * @param time extrinsics time if extrinsics are not static
   * @return true if lookup was successful
   */
  bool GetT_CAMERA_IMU(Eigen::Matrix4d& T,
                       const ros::Time& time = ros::Time(0));

  /**
   * @brief Gets the extrinsics between IMU and camera
   * @param T reference to result
   * @param time extrinsics time if extrinsics are not static
   * @return true if lookup was successful
   */
  bool GetT_IMU_CAMERA(Eigen::Matrix4d& T,
                       const ros::Time& time = ros::Time(0));

  /**
   * @brief Gets the extrinsics between camera and lidar
   * @param T reference to result
   * @param time extrinsics time if extrinsics are not static
   * @return true if lookup was successful
   */
  bool GetT_CAMERA_LIDAR(Eigen::Matrix4d& T,
                         const ros::Time& time = ros::Time(0));

  /**
   * @brief Gets the extrinsics between lidar and camera
   * @param T reference to result
   * @param time extrinsics time if extrinsics are not static
   * @return true if lookup was successful
   */
  bool GetT_LIDAR_CAMERA(Eigen::Matrix4d& T,
                         const ros::Time& time = ros::Time(0));

  /**
   * @brief Gets the extrinsics between IMU and lidar
   * @param T reference to result
   * @param time extrinsics time if extrinsics are not static
   * @return true if lookup was successful
   */
  bool GetT_IMU_LIDAR(Eigen::Matrix4d& T, const ros::Time& time = ros::Time(0));

  /**
   * @brief Gets the extrinsics between lidar and IMU
   * @param T reference to result
   * @param time extrinsics time if extrinsics are not static
   * @return true if lookup was successful
   */
  bool GetT_LIDAR_IMU(Eigen::Matrix4d& T, const ros::Time& time = ros::Time(0));

  /**
   * @brief Gets the extrinsics between baselink and IMU
   * @param T reference to result
   * @param time extrinsics time if extrinsics are not static
   * @return true if lookup was successful
   */
  bool GetT_BASELINK_IMU(Eigen::Matrix4d& T,
                         const ros::Time& time = ros::Time(0));

  /**
   * @brief Gets the extrinsics between baselink and camera
   * @param T reference to result
   * @param time extrinsics time if extrinsics are not static
   * @return true if lookup was successful
   */
  bool GetT_BASELINK_CAMERA(Eigen::Matrix4d& T,
                            const ros::Time& time = ros::Time(0));

  /**
   * @brief Gets the extrinsics between baselink and lidar
   * @param T reference to result
   * @param time extrinsics time if extrinsics are not static
   * @return true if lookup was successful
   */
  bool GetT_BASELINK_LIDAR(Eigen::Matrix4d& T,
                           const ros::Time& time = ros::Time(0));

  /**
   * @brief Gets the extrinsics between baselink and sensor
   * @param T reference to result
   * @param sensor_frame sensor frame id
   * @param time extrinsics time if extrinsics are not static
   * @return true if lookup was successful
   */
  bool GetT_BASELINK_SENSOR(Eigen::Matrix4d& T, const std::string& sensor_frame,
                            const ros::Time& time = ros::Time(0));

  /**
   * @brief Gets the extrinsics between IMU and baselink
   * @param T reference to result
   * @param time extrinsics time if extrinsics are not static
   * @return true if lookup was successful
   */
  bool GetT_IMU_BASELINK(Eigen::Matrix4d& T,
                         const ros::Time& time = ros::Time(0));

  /**
   * @brief Gets the extrinsics between camera and baselink
   * @param T reference to result
   * @param time extrinsics time if extrinsics are not static
   * @return true if lookup was successful
   */
  bool GetT_CAMERA_BASELINK(Eigen::Matrix4d& T,
                            const ros::Time& time = ros::Time(0));

  /**
   * @brief Gets the extrinsics between lidar and baselink
   * @param T reference to result
   * @param time extrinsics time if extrinsics are not static
   * @return true if lookup was successful
   */
  bool GetT_LIDAR_BASELINK(Eigen::Matrix4d& T,
                           const ros::Time& time = ros::Time(0));

  /**
   * @brief Gets the extrinsics between sensor and baselink
   * @param T reference to result
   * @param sensor_frame sensor frame id
   * @param time extrinsics time if extrinsics are not static
   * @return true if lookup was successful
   */
  bool GetT_SENSOR_BASELINK(Eigen::Matrix4d& T, const std::string& sensor_frame,
                            const ros::Time& time = ros::Time(0));

  /**
   * @brief Gets the frame id of IMU
   * @return frame id
   */
  std::string GetImuFrameId() const;

  /**
   * @brief Gets the frame id of camera
   * @return frame id
   */
  std::string GetCameraFrameId() const;

  /**
   * @brief Gets the frame id of lidar
   * @return frame id
   */
  std::string GetLidarFrameId() const;

  /**
   * @brief Gets the frame id of the world frame
   * @return frame id
   */
  std::string GetWorldFrameId() const;

  /**
   * @brief Gets the frame id of the baselink frame
   * @return frame id
   */
  std::string GetBaselinkFrameId() const;

  /**
   * @brief Gets the status on whether or not extrinsics are static
   * @return true if extrinsics are static
   */
  bool IsStatic() const;

  /**
   * @brief Verifies if sensor frame id is valid by checking against IMU,
   * camera, or lidar frame id
   * @return true if sensor frame id matches any of these frames
   */
  bool IsSensorFrameIdValid(const std::string& sensor_frame);

  /**
   * @brief get all stored frame ids
   * @return string of comma separated frame ids
   */
  std::string GetFrameIdsString();

private:
  /**
   * @brief Constructor
   */
  ExtrinsicsLookupOnline();

  /**
   * @brief Looks up the transform between specified frames using the
   * tf_listener
   * @param T reference to result
   * @param to_frame 'to frame' of transformation
   * @param from_frame 'from frame' of transformation
   * @param time extrinsics time if extrinsics are not static
   * @param max_iterations maximum times we should try to lookup transform
   * @param sleep_time how long to sleep in between iterations (pose lookups)
   * @return true if lookup was successful
   */
  bool LookupTransform(Eigen::Matrix4d& T, const std::string& to_frame,
                       const std::string& from_frame,
                       const ros::Time& time = ros::Time(0),
                       int max_iterations = 5,
                       const ros::Duration& sleep_time = ros::Duration(0.5));

  bs_parameters::models::CalibrationParams calibration_params_;

  tf::TransformListener tf_listener_;

  std::shared_ptr<ExtrinsicsLookupBase> extrinsics_;

  bool static_extrinsics_{true};
};

} // namespace bs_common
