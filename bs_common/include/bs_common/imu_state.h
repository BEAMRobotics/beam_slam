#pragma once

#include <Eigen/Eigen>
#include <fuse_core/graph.h>
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>
#include <fuse_variables/velocity_linear_3d_stamped.h>

#include <bs_common/preintegrator.h>
#include <bs_variables/accel_bias_3d_stamped.h>
#include <bs_variables/gyro_bias_3d_stamped.h>

namespace bs_common {

class ImuState {
public:
  /**
   * @brief default constructor
   */
  ImuState() = default;

  /**
   * @brief constructor when inputting time. Orientation is set to identity,
   * while all other variables are set to zero.
   * @param time timestamp for this imu state
   */
  ImuState(const ros::Time& time);

  /**
   * @brief constructor when inputting orientation, position, and velocity. Bias
   * variables are set to zero.
   * @param time timestamp for this imu state
   * @param orientation orientation for this imu state
   * @param position position for this imu state
   * @param velocity velocity for this imu state
   */
  ImuState(const ros::Time& time, const Eigen::Quaterniond& orientation,
           const Eigen::Vector3d& position, const Eigen::Vector3d& velocity);

  /**
   * @brief constructor when inputting orientation, position, and velocity. Bias
   * variables and velocity are set to zero.
   * @param time timestamp for this imu state
   * @param orientation orientation for this imu state
   * @param position position for this imu state
   * @param preint preintegrator for this state
   */
  ImuState(const ros::Time& time, const Eigen::Quaterniond& orientation,
           const Eigen::Vector3d& position,
           const bs_common::PreIntegrator& preint);

  /**
   * @brief constructor when inputting orientation, position, and velocity,
   * gyroscope bias, and acceleration bias
   * @param time timestamp for this imu state
   * @param orientation orientation for this imu state
   * @param position position for this imu state
   * @param velocity velocity for this imu state
   * @param gyrobias gyroscope bias for this imu state
   * @param accelbias acceleration bias for this imu state
   */
  ImuState(const ros::Time& time, const Eigen::Quaterniond& orientation,
           const Eigen::Vector3d& position, const Eigen::Vector3d& velocity,
           const Eigen::Vector3d& gyrobias, const Eigen::Vector3d& accelbias);

  /**
   * @brief update the velocity, pose, gyro bias and accel bias variables of
   * this ImuState given some graph message
   * @param graph_msg results from some optimizer which should contain the same
   * variable uuids that are stored herein
   * @return true update was successful (i.e., uuids were in the graph message)
   */
  bool Update(fuse_core::Graph::ConstSharedPtr graph_msg);

  /**
   * @brief update the velocity, gyro bias and accel bias variables of this
   * ImuState given some graph message
   * @param graph_msg results from some optimizer which should contain the same
   * variable uuids that are stored herein
   * @return true update was successful (i.e., uuids were in the graph message)
   */
  bool UpdateRelative(fuse_core::Graph::ConstSharedPtr graph_msg);

  /**
   * @brief get the number of times this ImuState has its variables updated by
   * some graph optimizer
   * @return number of variable updates
   */

  int Updates() const;

  /**
   * @brief return timestamp associated with this ImuState
   * @return stamp
   */
  ros::Time Stamp() const;

  /**
   * @brief Set the timestamp of this state
   */
  void SetStamp(const ros::Time stamp);

  /**
   * @brief return the current estimate of the orientation
   * @return orientation fuse variable
   */
  fuse_variables::Orientation3DStamped Orientation() const;

  /**
   * @brief return the current estimate of the orientation
   * @return orientation as a quaternion
   */
  Eigen::Quaterniond OrientationQuat() const;

  /**
   * @brief return the current estimate of the orientation
   * @return orientation as a rotation matrix
   */
  Eigen::Matrix3d OrientationMat() const;

  /**
   * @brief return the current estimate of the position
   * @return position fuse variable
   */
  fuse_variables::Position3DStamped Position() const;

  /**
   * @brief return the current estimate of the position
   * @return position vector
   */
  Eigen::Vector3d PositionVec() const;

  /**
   * @brief return the current estimate of the velocity
   * @return velocity fuse variable
   */
  fuse_variables::VelocityLinear3DStamped Velocity() const;

  /**
   * @brief return the current estimate of the velocity
   * @return velocity vector
   */
  Eigen::Vector3d VelocityVec() const;

  /**
   * @brief return the current estimate of the gyroscope bias
   * @return gyroscope bias beam variable
   */
  bs_variables::GyroscopeBias3DStamped GyroBias() const;

  /**
   * @brief return the current estimate of the gyroscope bias
   * @return gyroscope bias vector
   */
  Eigen::Vector3d GyroBiasVec() const;

  /**
   * @brief return the current estimate of the acceleration bias
   * @return acceleration bias beam variable
   */
  bs_variables::AccelerationBias3DStamped AccelBias() const;

  /**
   * @brief return the current estimate of the acceleration bias
   * @return acceleration bias vector
   */
  Eigen::Vector3d AccelBiasVec() const;

  /**
   * @brief get the preintegrator
   */
  bs_common::PreIntegrator& GetPreintegratorMutable();

  /**
   * @brief get the preintegrator (const reference)
   */
  const bs_common::PreIntegrator GetPreintegratorConst() const;

  /**
   * @brief set the preintegrator
   */
  void SetPreintegrator(const bs_common::PreIntegrator& preint);

  /**
   * @brief set orientation using double data type
   */
  void SetOrientation(const double& w, const double& x, const double& y,
                      const double& z);

  /**
   * @brief set orientation using Eigen::Quaterniond data type
   */
  void SetOrientation(const Eigen::Quaterniond& orientation);

  /**
   * @brief set orientation using c-style array
   */
  void SetOrientation(const double* orientation);

  /**
   * @brief set orientation using fuse variable
   */
  void SetOrientation(const fuse_variables::Orientation3DStamped orientation);

  /**
   * @brief set position using double data type
   */
  void SetPosition(const double& x, const double& y, const double& z);

  /**
   * @brief set position using Eigen::Vector3d data type
   */
  void SetPosition(const Eigen::Vector3d& position);

  /**
   * @brief set position using c-style array
   */
  void SetPosition(const double* position);

  /**
   * @brief set position using fuse variable
   */
  void SetPosition(const fuse_variables::Position3DStamped position);

  /**
   * @brief set velocity using double data type
   */
  void SetVelocity(const double& x, const double& y, const double& z);

  /**
   * @brief set velocity using Eigen::Vector3d data type
   */
  void SetVelocity(const Eigen::Vector3d& velocity);

  /**
   * @brief set velocity using c-style array
   */
  void SetVelocity(const double* velocity);

  /**
   * @brief set velocity using fuse variable
   */
  void SetVelocity(const fuse_variables::VelocityLinear3DStamped velocity);

  /**
   * @brief set gyroscope bias using double data type
   */
  void SetGyroBias(const double& x, const double& y, const double& z);

  /**
   * @brief set gyroscope bias using Eigen::Vector3d data type
   */
  void SetGyroBias(const Eigen::Vector3d& gyrobias);

  /**
   * @brief set gyroscope bias using c-style array
   */
  void SetGyroBias(const double* gyrobias);

  /**
   * @brief set gyroscope bias using fuse variable
   */
  void SetGyroBias(const bs_variables::GyroscopeBias3DStamped gyrobias);

  /**
   * @brief set acceleration bias using double data type
   */
  void SetAccelBias(const double& x, const double& y, const double& z);

  /**
   * @brief set acceleration bias using Eigen::Vector3d data type
   */
  void SetAccelBias(const Eigen::Vector3d& accelbias);

  /**
   * @brief set acceleration bias using c-style array
   */
  void SetAccelBias(const double* accelbias);

  /**
   * @brief set acceleration bias using fuse variable
   */
  void SetAccelBias(const bs_variables::AccelerationBias3DStamped accelbias);

  /**
   * @brief print relevant information about what is currently contained in this
   * ImuState.
   * @param stream input stream
   */
  void Print(std::ostream& stream = std::cout) const;

  /**
   * @brief deep copy of the member variables
   * @return copy of data
   */
  ImuState Copy() const;

private:
  int updates_{0};
  ros::Time stamp_;
  fuse_variables::Orientation3DStamped orientation_;
  fuse_variables::Position3DStamped position_;
  fuse_variables::VelocityLinear3DStamped velocity_;
  bs_variables::GyroscopeBias3DStamped gyrobias_;
  bs_variables::AccelerationBias3DStamped accelbias_;
  bs_common::PreIntegrator preint_;
};

} // namespace bs_common
