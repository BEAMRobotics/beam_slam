#pragma once

#include <Eigen/Dense>
#include <tf/transform_listener.h>
#include <tf2/buffer_core.h>

namespace beam_common {

/**
 * @brief This class can be used to estimate a pose of any frame given its
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
   * @param poses shared pointer to the poses {T_WORLD_BASELINK} [REQUIRED]
   * @param sensor_frame_id frame ID attached to the sensor, used to lookup
   * extrinsic calibrations. If not supplied, it will assume the odometry is
   * already in the correct frame. [OPTIONAL]
   * @param baselink_frame moving frame in the poses [REQUIRED]
   * @param world_frame fixed frame in the poses [REQUIRED]
   * @param static_extrinsics set to true if the extrinsic calibrations are time
   * invariant [OPTIONAL]
   */
  struct Params {
    std::shared_ptr<tf2::BufferCore> poses{nullptr};
    std::string sensor_frame{""};
    std::string baselink_frame{""};
    std::string world_frame{""};
    bool static_extrinsics{true};
  };

  /**
   * @brief Constructor
   * @param params all input params required. See struct above
   */
  PoseLookup(const Params& params);

  /**
   * @brief Gets estimate frame pose
   * @param time stamp of the frame being initialized
   * @param T_WORLD_SENSOR reference to result
   * @return true if pose lookup was successful
   */
  bool GetT_WORLD_SENSOR(Eigen::Matrix4d& T_WORLD_SENSOR,
                         const ros::Time& time);

  /**
   * @brief Gets extrinsics
   * @param T_WORLD_SENSOR reference to result
   * @param time extrinsic time to lookup. If empty, or set to ros::Time(0), it
   * will use most recent available. This is also used when static_extrinsics is
   * set to true
   * @return true if extrinsics lookup was successful
   */
  bool GetT_BASELINK_SENSOR(Eigen::Matrix4d& T_WORLD_BASELINK,
                            const ros::Time& time = ros::Time(0));

  /**
   * @brief Gets pose
   * @param T_WORLD_BASELINK reference to result
   * @param time pose time to lookup.
   * @return true if pose lookup was successful
   */
  bool GetT_WORLD_BASELINK(Eigen::Matrix4d& T_WORLD_BASELINK,
                           const ros::Time& time);

private:
  Params params_;
  tf::TransformListener tf_listener_;

  bool extrinsics_set_{false};
  Eigen::Matrix4d T_BASELINK_SENSOR_;
};

} // namespace beam_common
