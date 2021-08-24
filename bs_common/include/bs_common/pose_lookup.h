#pragma once

#include <Eigen/Dense>
#include <tf2/buffer_core.h>

#include <bs_common/extrinsics_lookup_online.h>

namespace bs_common {

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
   * @brief Constructor
   * @param poses shared pointer to the poses
   */
  PoseLookup(const std::shared_ptr<tf2::BufferCore> poses);

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
   * @brief Gets estimate of baselink frame pose wrt world frame
   * @param T_WORLD_BASELINK reference to result
   * @param time pose time to lookup.
   * @return true if pose lookup was successful
   */
  bool GetT_WORLD_BASELINK(Eigen::Matrix4d& T_WORLD_BASELINK,
                           const ros::Time& time);

 private:
  bs_common::ExtrinsicsLookupOnline& extrinsics_ =
      bs_common::ExtrinsicsLookupOnline::GetInstance();

  std::shared_ptr<tf2::BufferCore> poses_{nullptr};
};

}  // namespace bs_common
