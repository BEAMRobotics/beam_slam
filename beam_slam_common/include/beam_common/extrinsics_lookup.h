#pragma once

#include <Eigen/Dense>
#include <tf/transform_listener.h>

namespace beam_common {

/**
 * @brief this class can be used to lookup static or dynamic extrinsics
 * calibrations which are being published to tf or tf_static. The 3 types of
 * frames need to be supplied to this class. The prefered way to use this with
 * beam_slam is to add global variables to the config yaml for each of the
 * frames, then each of the sensor models can instantiate this class with those
 * same global params. See global_mapper.cpp for an example use case.
 *
 */
class ExtrinsicsLookup {
 public:
  /**
   * @param imu_frame frame ID attached to the imu sensor [REQUIRED]
   * @param camera_frame frame ID attached to the camera sensor [REQUIRED]
   * @param lidar_frame frame ID attached to the lidar sensor [REQUIRED]
   * @param static_extrinsics set to true if the extrinsic calibrations are time
   * invariant [OPTIONAL]
   */
  struct Params {
    std::string imu_frame{""};
    std::string camera_frame{""};
    std::string lidar_frame{""};
    bool static_extrinsics{true};
  };

  /**
   * @brief Static Instance getter (singleton)
   * @return reference to the singleton
   */
  static ExtrinsicsLookup& GetInstance();

  /**
   * @brief Gets the extrinsics between camera and imu
   * @param time extrinsics time if extrinsics are not static
   * @param T reference to result
   * @return true if lookup was successful
   */
  bool GetT_CAMERA_IMU(Eigen::Matrix4d& T,
                       const ros::Time& time = ros::Time(0));

  /**
   * @brief Gets the extrinsics between camera and imu
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
   * @brief Gets the extrinsics between camera and lidar
   * @param time extrinsics time if extrinsics are not static
   * @param T reference to result
   * @return true if lookup was successful
   */
  bool GetT_LIDAR_CAMERA(Eigen::Matrix4d& T,
                         const ros::Time& time = ros::Time(0));

  /**
   * @brief Gets the extrinsics between lidar and IMU
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

  Params params;

 private:
  /**
   * @brief Constructor
   * @param params all input params required. See struct above
   */
  ExtrinsicsLookup(const Params& params);

  /**
   * @brief default constructor
   */
  ExtrinsicsLookup() = delete;

  /**
   * @brief copy constructor
   */
  ExtrinsicsLookup(const ExtrinsicsLookup&);

  /**
   * @brief copy assignment operator
   */
  ExtrinsicsLookup& operator=(const ExtrinsicsLookup&);

  static ExtrinsicsLookup& instance_;

  bool GetTransform(Eigen::Matrix4d& T, const std::string& to_frame,
                    const std::string& from_frame, const ros::Time& time);

  tf::TransformListener tf_listener_;

  fuse_variables::Position3DStamped imu_position;     // identity
  fuse_variables::Position3DStamped camera_position;  // t_BASELINK_CAMERA
  fuse_variables::Position3DStamped lidar_position;   // t_BASELINK_LIDAR

  fuse_variables::Orientation3DStamped imu_orientation;
  fuse_variables::Orientation3DStamped camera_orientation;  // R_BASELINK_CAMERA
  fuse_variables::Orientation3DStamped lidar_orientation;   // R_BASELINK_LIDAR

  Eigen::Matrix4d T_LIDAR_IMU_;
  Eigen::Matrix4d T_LIDAR_CAMERA_;
  Eigen::Matrix4d T_IMU_CAMERA_;

  bool T_LIDAR_IMU_set_{false};
  bool T_LIDAR_CAMERA_set_{false};
  bool T_IMU_CAMERA_set_{false};
};

}  // namespace beam_common
