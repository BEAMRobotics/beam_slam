#pragma once

#include <fuse_core/graph.h>
#include <fuse_core/transaction.h>
#include <fuse_core/variable.h>
#include <fuse_variables/acceleration_linear_3d_stamped.h>
#include <fuse_variables/velocity_angular_3d_stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>

#include <beam_utils/math.h>
#include <beam_utils/se3.h>

#include <bs_common/imu_state.h>

#ifndef GRAVITY_NOMINAL
#  define GRAVITY_NOMINAL 9.80665
#endif

static const Eigen::Vector3d GRAVITY_WORLD{0.0, 0.0, -GRAVITY_NOMINAL};

namespace bs_common {

template <typename T>
inline Eigen::Quaternion<T> DeltaQ(const Eigen::Matrix<T, 3, 1>& theta) {
  Eigen::Quaternion<T> dq;
  Eigen::Matrix<T, 3, 1> half_theta = theta;
  half_theta /= static_cast<T>(2.0);
  dq.w() = static_cast<T>(1.0);
  dq.x() = half_theta(0, 0);
  dq.y() = half_theta(1, 0);
  dq.z() = half_theta(2, 0);
  return dq;
}

std::string ToString(const ros::Time& time);

void InterpolateTransformFromPath(
    const std::vector<geometry_msgs::PoseStamped>& poses, const ros::Time& time,
    Eigen::Matrix4d& T_WORLD_SENSOR);

/**
 * @brief Estimates a velocity at a time given a set of poses
 * @param poses list of poses
 * @param time time to interpolate pose for
 * @param velocity to return
 */
void EstimateVelocityFromPath(
    const std::vector<geometry_msgs::PoseStamped>& poses, const ros::Time& time,
    Eigen::Vector3d& velocity);

/**
 * @brief Get full path the the config root directory in beam_slam_launch
 * (.../beam_slam_launch/config/)
 * @return path
 */
std::string GetBeamSlamConfigPath();

/**
 * @brief Get full path the the calibrations root directory in beam_slam_launch
 * (.../beam_slam_launch/calibrations/)
 * @return path
 */
std::string GetBeamSlamCalibrationsPath();

/**
 * @brief Shannon Entropy: This can be interpreted geometrically as the volume
 * of the uncertainty. The smaller the Shannon Entropy, the more certain we are.
 *
 * H(x) = 0.5 ln[(2 PI e)^N det(cov)]
 *
 * Where N is the numer of parameters being estimated
 *
 */
double ShannonEntropyFromPoseCovariance(
    const Eigen::Matrix<double, 6, 6>& covariance);

} // namespace bs_common
