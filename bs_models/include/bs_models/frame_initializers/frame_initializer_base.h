#pragma once

#include <Eigen/Dense>
#include <ros/ros.h>
#include <tf2/buffer_core.h>

#include <bs_common/pose_lookup.h>

namespace bs_models { namespace frame_initializers {

static std::string frame_initializer_error_msg = "";

/**
 * @brief This base class shows the contract between a FrameInitializer class.
 * The goal of this class is to initialize the pose of a frame given some
 * timestamp. This can simply be from a published topic, or can use an odometry
 * methodology with input sensor data. For more information on frames, see the
 * PoseLookup and ExtrinsicsLookupOnline classes.
 *
 * All input data to the derived classes should be added in a custom
 * constructor. The constructor also needs to initialize pose_lookup_ and poses_
 *
 */
class FrameInitializerBase {
public:
  /**
   * @brief Gets estimated pose of sensor frame wrt world frame using
   * Poselookup.
   * @param T_WORLD_SENSOR reference to result
   * @param time stamp of the frame being initialized
   * @param sensor_frame sensor frame id.
   * @return true if pose lookup was successful
   */
  bool GetPose(Eigen::Matrix4d& T_WORLD_SENSOR, const ros::Time& time,
               const std::string& sensor_frame_id,
               std::string& error_msg = frame_initializer_error_msg);

  /**
   * @brief Gets relative pose between two timestamps wrt the world frame
   * @param T_A_B [out] relative pose
   * @param tA [in] time at state A
   * @param tB [in] time at state B
   * @return true if pose lookup was successful
   */
  bool GetRelativePose(Eigen::Matrix4d& T_A_B, const ros::Time& tA,
                       const ros::Time& tB,
                       std::string& error_msg = frame_initializer_error_msg);

  /**
   * @brief Factory method for creating a Frame initializer from a json config
   * @param config_path path to json config
   * @return unique ptr to a frame initializer
   */
  static std::unique_ptr<frame_initializers::FrameInitializerBase>
      Create(const std::string& config_path);

protected:
  std::string authority_;
  std::shared_ptr<bs_common::PoseLookup> pose_lookup_;
  bs_common::ExtrinsicsLookupOnline& extrinsics_ =
      bs_common::ExtrinsicsLookupOnline::GetInstance();
};

}} // namespace bs_models::frame_initializers