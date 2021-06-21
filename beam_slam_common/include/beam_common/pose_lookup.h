#pragma once

#include <Eigen/Dense>
#include <tf/transform_listener.h>
#include <tf2/buffer_core.h>

#include <beam_common/extrinsics_lookup.h>

namespace beam_common {

/**
 * @brief This class can be used to estimate the pose of any frame given its
 * timestamp and a tf2::BufferCore which contains the poses.
 *
 * Frames: Extrinsic calibration must be available on tf. Extrinsics can be
 * static or dynamic. To lookup the transforms needed, the sensor frame id must
 * be supplied, if it is not supplied then it will assume the odometry is
 * already in the correct frame.
 *
 */
class PoseLookup {
 public:
  /**
   * @brief Static Instance getter (singleton)
   * @return reference to the singleton
   */
  static PoseLookup& GetInstance();

  /**
   * @brief Copy constructor
   */
  PoseLookup(const PoseLookup& other) = delete;

  /**
   * @brief Copy assignment operator
   */
  PoseLookup& operator=(const PoseLookup& other) = delete;

  /**
   * @brief Set Poses as a shared pointer. Setting poses has been enforced
   * @return true if poses are set
   */
  bool SetPoses(const std::shared_ptr<tf2::BufferCore> poses);

  /**
   * @brief Gets estimate of sensor frame pose wrt world frame
   * @param T_WORLD_SENSOR reference to result
   * @param sensor_frame sensor frame id
   * @param time stamp of the frame being initialized
   * @return true if pose lookup was successful
   */
  bool GetT_WORLD_SENSOR(Eigen::Matrix4d& T_WORLD_SENSOR,
                         const std::string& sensor_frame,
                         const ros::Time& time);

  /**
   * @brief Gets estimate of sensor frame pose wrt baselink frame using
   * ExtrinsicsLookup
   * @param T_BASELINK_SENSOR reference to result
   * @param sensor_frame sensor frame id. If empty, the assumed sensor frame
   * will be the baselink frame
   * @param time extrinsic time to lookup. If empty, or set to ros::Time(0), it
   * will use most recent available. This is also used when static_extrinsics is
   * set to true
   * @return true if extrinsics lookup was successful
   */
  bool GetT_BASELINK_SENSOR(Eigen::Matrix4d& T_BASELINK_SENSOR,
                            const std::string& sensor_frame = "",
                            const ros::Time& time = ros::Time(0));

  /**
   * @brief Gets estimate of baselink frame pose wrt world frame
   * @param T_WORLD_BASELINK reference to result
   * @param time pose time to lookup.
   * @return true if pose lookup was successful
   */
  bool GetT_WORLD_BASELINK(Eigen::Matrix4d& T_WORLD_BASELINK,
                           const ros::Time& time);

  /**
   * @brief Gets the frame id of world
   * @return frame id
   */
  std::string GetWorldFrameID() const { return world_frame_; }

  /**
   * @brief Gets the frame id of baselink
   * @return frame id
   */
  std::string GetBaselinkFrameID() const { return baselink_frame_; }

 private:
  /**
   * @brief Constructor
   */
  PoseLookup();

  /**
   * @brief Check to ensure that poses has been set
   */
  void CheckPoses();

  /**
   * @brief Commonly thrown error when incorrectly getting baselink to sensor
   * transform
   */
  bool ThrowFrameIDError();

  beam_common::ExtrinsicsLookup& extrinsics_ =
      beam_common::ExtrinsicsLookup::GetInstance();

  tf::TransformListener tf_listener_;

  std::string world_frame_{""};
  std::string baselink_frame_{""};

  std::shared_ptr<tf2::BufferCore> poses_{nullptr};
};

}  // namespace beam_common
