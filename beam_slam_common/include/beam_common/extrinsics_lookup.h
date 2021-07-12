#pragma once

#include <Eigen/Dense>
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>
#include <tf/transform_listener.h>

namespace beam_common {

/**
 * @brief this class can be used to lookup static or dynamic extrinsics
 * calibrations which are being published to tf or tf_static. The 3 types of
 * frames need to be supplied to this class. The prefered way to use this with
 * beam_slam is to add global variables to the config yaml for each of the
 * frames, then each of the sensor models can get the instance of this class
 * with those same global params. See global_mapper.cpp for an example use case.
 */
class ExtrinsicsLookup {
 public:
  /**
   * @brief Static Instance getter (singleton)
   * @return reference to the singleton
   */
  static ExtrinsicsLookup& GetInstance();

  /**
   * @brief Copy constructor
   */
  ExtrinsicsLookup(const ExtrinsicsLookup& other) = delete;

  /**
   * @brief Copy assignment operator
   */
  ExtrinsicsLookup& operator=(const ExtrinsicsLookup& other) = delete;

  /**
   * @brief Gets the extrinsics between camera and imu
   * @param time extrinsics time if extrinsics are not static
   * @param T reference to result
   * @return true if lookup was successful
   */
  bool GetT_CAMERA_IMU(Eigen::Matrix4d& T,
                       const ros::Time& time = ros::Time(0));

  /**
   * @brief Gets the extrinsics between imu and camera
   * @param time extrinsics time if extrinsics are not static
   * @param T reference to result
   * @return true if lookup was successful
   */
  bool GetT_IMU_CAMERA(Eigen::Matrix4d& T,
                       const ros::Time& time = ros::Time(0));

  /**
   * @brief Gets the extrinsics between camera and lidar
   * @param time extrinsics time if extrinsics are not static
   * @param T reference to result
   * @return true if lookup was successful
   */
  bool GetT_CAMERA_LIDAR(Eigen::Matrix4d& T,
                         const ros::Time& time = ros::Time(0));

  /**
   * @brief Gets the extrinsics between lidar and camera
   * @param time extrinsics time if extrinsics are not static
   * @param T reference to result
   * @return true if lookup was successful
   */
  bool GetT_LIDAR_CAMERA(Eigen::Matrix4d& T,
                         const ros::Time& time = ros::Time(0));

  /**
   * @brief Gets the extrinsics between IMU and lidar
   * @param time extrinsics time if extrinsics are not static
   * @param T reference to result
   * @return true if lookup was successful
   */
  bool GetT_IMU_LIDAR(Eigen::Matrix4d& T, const ros::Time& time = ros::Time(0));

  /**
   * @brief Gets the extrinsics between lidar and IMU
   * @param time extrinsics time if extrinsics are not static
   * @param T reference to result
   * @return true if lookup was successful
   */
  bool GetT_LIDAR_IMU(Eigen::Matrix4d& T, const ros::Time& time = ros::Time(0));

  /**
   * @brief Gets the frame id of IMU
   * @return frame id
   */
  std::string GetIMUFrameID() const { return imu_frame_; }

  /**
   * @brief Gets the frame id of camera
   * @return frame id
   */
  std::string GetCameraFrameID() const { return camera_frame_; }

  /**
   * @brief Gets the frame id of lidar
   * @return frame id
   */
  std::string GetLidarFrameID() const { return lidar_frame_; }

  /**
   * @brief Gets the status on whether or not extrinsics are static
   * @return true if extrinsics are static
   */
  bool IsStatic() const { return static_extrinsics_; }

  /**
   * @brief Gets the status on whether or not extrinsics between all frames have
   * been set
   * @return true if extrinsics between all frames have been set
   */
  bool IsSet() const {
    return T_LIDAR_IMU_set_ && T_LIDAR_CAMERA_set_ && T_IMU_CAMERA_set_;
  }

 private:
  /**
   * @brief Constructor
   */
  ExtrinsicsLookup();

  /**
   * @brief Gets transform between specified frames
   * @param T reference to result
   * @param to_frame 'to frame' of transformation
   * @param from_frame 'from frame' of transformation
   * @param time extrinsics time if extrinsics are not static
   * @return true if lookup was successful
   */
  bool GetTransform(Eigen::Matrix4d& T, const std::string& to_frame,
                    const std::string& from_frame, const ros::Time& time);

  tf::TransformListener tf_listener_;

  fuse_variables::Position3DStamped imu_position_;     // zero position
  fuse_variables::Position3DStamped camera_position_;  // t_BASELINK_CAMERA
  fuse_variables::Position3DStamped lidar_position_;   // t_BASELINK_LIDAR

  fuse_variables::Orientation3DStamped imu_orientation_;  // zero rotation
  fuse_variables::Orientation3DStamped
      camera_orientation_;                                  // R_BASELINK_CAMERA
  fuse_variables::Orientation3DStamped lidar_orientation_;  // R_BASELINK_LIDAR

  Eigen::Matrix4d T_LIDAR_IMU_;
  Eigen::Matrix4d T_LIDAR_CAMERA_;
  Eigen::Matrix4d T_IMU_CAMERA_;

  std::string imu_frame_{""};
  std::string camera_frame_{""};
  std::string lidar_frame_{""};

  bool T_LIDAR_IMU_set_{false};
  bool T_LIDAR_CAMERA_set_{false};
  bool T_IMU_CAMERA_set_{false};
  bool static_extrinsics_{true};
};

}  // namespace beam_common
