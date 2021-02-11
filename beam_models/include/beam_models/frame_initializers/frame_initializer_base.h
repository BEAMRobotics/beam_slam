#pragma once

#include <ros/ros.h>
#include <Eigen/Dense>

namespace beam_models { namespace frame_initializers {

/**
 * @brief This base class shows the contract between a FrameInitializer class.
 * The goal of this class is to initialize the pose of a frame given some
 * timestamp. This can simply be from a published topic, or can use an odometry
 * methodology with input sensor data.
 *
 * All input data to the derived classes should be added in a custom constructor
 *
 */
class FrameInitializerBase {
public:
  /**
   * @brief Pure virtual class for getting the estimated frame pose
   * @param time stamp of the frame being initialized
   * @param T_WORLD_SENSOR reference to result
   * @return true if pose lookup was successful
   */
  virtual bool GetEstimatedPose(const ros::Time& time, Eigen::Matrix4d& T_WORLD_SENSOR) = 0;

protected:
  // NULL
};

}} // namespace beam_models::frame_initializers