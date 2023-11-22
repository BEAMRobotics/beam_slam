#pragma once

#include <Eigen/Dense>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>

#include <bs_variables/orientation_3d.h>
#include <bs_variables/position_3d.h>
#include <fuse_core/eigen.h>
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>

namespace bs_common {

// returns [tx, ty, tz, qw, qx, qy, qz]^T
Eigen::Matrix<double, 7, 1>
    TransformMatrixToVectorWithQuaternion(const Eigen::Matrix4d& T);

void EigenTransformToFusePose(const Eigen::Matrix4d& T_WORLD_SENSOR,
                              fuse_variables::Position3DStamped& p,
                              fuse_variables::Orientation3DStamped& o);

void EigenTransformToFusePose(const Eigen::Matrix4d& T_WORLD_SENSOR,
                              bs_variables::Position3D& p,
                              bs_variables::Orientation3D& o);

void FusePoseToEigenTransform(const fuse_variables::Position3DStamped& p,
                              const fuse_variables::Orientation3DStamped& o,
                              Eigen::Matrix4d& T_WORLD_SENSOR);
Eigen::Matrix4d
    FusePoseToEigenTransform(const fuse_variables::Position3DStamped& p,
                             const fuse_variables::Orientation3DStamped& o);

void FusePoseToEigenTransform(const bs_variables::Position3D& p,
                              const bs_variables::Orientation3D& o,
                              Eigen::Matrix4d& T_WORLD_SENSOR);

Eigen::Matrix4d FusePoseToEigenTransform(const bs_variables::Position3D& p,
                                         const bs_variables::Orientation3D& o);

void PoseMsgToTransformationMatrix(const geometry_msgs::PoseStamped& pose,
                                   Eigen::Matrix4d& T_WORLD_SENSOR);

void OdometryMsgToTransformationMatrix(const nav_msgs::Odometry& odom,
                                       Eigen::Matrix4d& T_WORLD_SENSOR);

void ROSStampedTransformToEigenTransform(const tf::StampedTransform& TROS,
                                         Eigen::Matrix4d& T);

void TransformStampedMsgToEigenTransform(
    const geometry_msgs::TransformStamped& TROS, Eigen::Matrix4d& T);

void EigenTransformToTransformStampedMsg(
    const Eigen::Matrix4d& T, const ros::Time& stamp, int seq,
    const std::string& parent_frame_id, const std::string& child_frame_id,
    geometry_msgs::TransformStamped& tf_stamped);

void EigenTransformToOdometryMsg(const Eigen::Matrix4d& T,
                                 const ros::Time& stamp, int seq,
                                 const std::string& parent_frame_id,
                                 const std::string& child_frame_id,
                                 nav_msgs::Odometry& odom_msg);
void EigenTransformToPoseStamped(const Eigen::Matrix4d& T,
                                 const ros::Time& stamp, int seq,
                                 const std::string& frame_id,
                                 geometry_msgs::PoseStamped& pose_stamped);

void OdometryMsgToTransformedStamped(
    const nav_msgs::Odometry& message, const ros::Time& stamp, int seq,
    const std::string& parent_frame_id, const std::string& child_frame_id,
    geometry_msgs::TransformStamped& tf_stamped);

nav_msgs::Odometry TransformToOdometryMessage(
    const ros::Time& stamp, const int seq, const std::string& parent_frame_id,
    const std::string& child_frame_id, const Eigen::Matrix4d T_PARENT_CHILD,
    const Eigen::Matrix<double, 6, 6> covariance =
        Eigen::Matrix<double, 6, 6>::Identity());

fuse_core::Vector7d ComputeDelta(const Eigen::Matrix4d& T_A_B);

/**
 * @brief Turns a pose message into an Eigen 4x4 matrix
 * @param pose pose message to turn into eigen matrix
 * @param T_WORLD_SENSOR[out] Transform to return
 */
void TransformationMatrixToPoseMsg(const Eigen::Matrix4d& T_WORLD_SENSOR,
                                   const ros::Time& stamp,
                                   geometry_msgs::PoseStamped& pose);

template <typename Variable_t>
Eigen::VectorXd FixedSizeVariableToEigen(const Variable_t& fuse_variable) {
  Eigen::VectorXd vec(fuse_variable.SIZE);
  for (int i = 0; i < fuse_variable.SIZE; i++) {
    vec << fuse_variable.data()[0];
  }
  return vec;
}

Eigen::Quaterniond OrientationVariableToEigenQuaternion(
    const fuse_variables::Orientation3DStamped& orientation);

} // namespace bs_common
