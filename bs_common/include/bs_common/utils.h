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
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_datatypes.h>

#include <beam_utils/math.h>
#include <beam_utils/se3.h>

#include <bs_common/imu_state.h>

#ifndef GRAVITY_NOMINAL
#  define GRAVITY_NOMINAL 9.80665
#endif

static const Eigen::Vector3d GRAVITY_WORLD{0.0, 0.0, GRAVITY_NOMINAL};

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

// Draw a coordinate frame with a velocity vector.
// For the frame: RGB corresponds to the frames (x-r, y-g, z-b) and label
// corresponds to the timestamp For the velocity vector: RGB is always magenta,
// label corresponds to the magnitude of velocity in mm/s (m/s * 1000)
pcl::PointCloud<pcl::PointXYZRGBL>
    ImuStateToCloudInWorld(const ImuState& imu_state);

/// @brief
/// @param T_WORLD_SENSOR
/// @param p
/// @param o
void EigenTransformToFusePose(const Eigen::Matrix4d& T_WORLD_SENSOR,
                              fuse_variables::Position3DStamped& p,
                              fuse_variables::Orientation3DStamped& o);

/// @brief
/// @param T_WORLD_SENSOR
/// @param p
/// @param o
void EigenTransformToFusePose(
    const Eigen::Matrix4d& T_WORLD_SENSOR,
    fuse_variables::Position3DStamped::SharedPtr& p,
    fuse_variables::Orientation3DStamped::SharedPtr& o);

/// @brief
/// @param p
/// @param o
/// @param T_WORLD_SENSOR
void FusePoseToEigenTransform(const fuse_variables::Position3DStamped& p,
                              const fuse_variables::Orientation3DStamped& o,
                              Eigen::Matrix4d& T_WORLD_SENSOR);
/// @brief
/// @param p
/// @param o
/// @return
Eigen::Matrix4d
    FusePoseToEigenTransform(const fuse_variables::Position3DStamped& p,
                             const fuse_variables::Orientation3DStamped& o);

/// @brief
/// @param pose
/// @param T_WORLD_SENSOR
void PoseMsgToTransformationMatrix(const geometry_msgs::PoseStamped& pose,
                                   Eigen::Matrix4d& T_WORLD_SENSOR);

/// @brief
/// @param odom
/// @param T_WORLD_SENSOR
void OdometryMsgToTransformationMatrix(const nav_msgs::Odometry& odom,
                                       Eigen::Matrix4d& T_WORLD_SENSOR);

/// @brief
/// @param TROS
/// @param T
void ROSStampedTransformToEigenTransform(const tf::StampedTransform& TROS,
                                         Eigen::Matrix4d& T);

/// @brief
/// @param TROS
/// @param T
void TransformStampedMsgToEigenTransform(
    const geometry_msgs::TransformStamped& TROS, Eigen::Matrix4d& T);

/// @brief
/// @param T
/// @param stamp
/// @param seq
/// @param parent_frame_id
/// @param child_frame_id
/// @param tf_stamped
void EigenTransformToTransformStampedMsg(
    const Eigen::Matrix4d& T, const ros::Time& stamp, int seq,
    const std::string& parent_frame_id, const std::string& child_frame_id,
    geometry_msgs::TransformStamped& tf_stamped);

/// @brief
/// @param T
/// @param stamp
/// @param seq
/// @param parent_frame_id
/// @param child_frame_id
/// @param odom_msg
void EigenTransformToOdometryMsg(const Eigen::Matrix4d& T,
                                 const ros::Time& stamp, int seq,
                                 const std::string& parent_frame_id,
                                 const std::string& child_frame_id,
                                 nav_msgs::Odometry& odom_msg);
/// @brief
/// @param T
/// @param stamp
/// @param seq
/// @param frame_id
/// @param pose_stamped
void EigenTransformToPoseStamped(const Eigen::Matrix4d& T,
                                 const ros::Time& stamp, int seq,
                                 const std::string& frame_id,
                                 geometry_msgs::PoseStamped& pose_stamped);

/// @brief
/// @param message
/// @param stamp
/// @param seq
/// @param parent_frame_id
/// @param child_frame_id
/// @param tf_stamped
void OdometryMsgToTransformedStamped(
    const nav_msgs::Odometry& message, const ros::Time& stamp, int seq,
    const std::string& parent_frame_id, const std::string& child_frame_id,
    geometry_msgs::TransformStamped& tf_stamped);

/// @brief
/// @param stamp
/// @param seq
/// @param parent_frame_id
/// @param child_frame_id
/// @param T_PARENT_CHILD
/// @param covariance
/// @return
nav_msgs::Odometry TransformToOdometryMessage(
    const ros::Time& stamp, const int seq, const std::string& parent_frame_id,
    const std::string& child_frame_id, const Eigen::Matrix4d T_PARENT_CHILD,
    const Eigen::Matrix<double, 6, 6> covariance =
        Eigen::Matrix<double, 6, 6>::Identity());

/**
 * @brief Turns a pose message into an Eigen 4x4 matrix
 * @param pose pose message to turn into eigen matrix
 * @param T_WORLD_SENSOR[out] Transform to return
 */
void TransformationMatrixToPoseMsg(const Eigen::Matrix4d& T_WORLD_SENSOR,
                                   const ros::Time& stamp,
                                   geometry_msgs::PoseStamped& pose);

/**
 * @brief Interpolates a pose given a list of poses and a time
 * @param poses list of poses
 * @param time time to interpolate pose for
 * @param T_WORLD_SENSOR[out] pose to return
 */
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
 * @brief Get number of constraints being added by a transaction
 * @param transaction
 * @return number of constraints
 */
int GetNumberOfConstraints(
    const fuse_core::Transaction::SharedPtr& transaction);

/**
 * @brief Get number of variables being added by a transaction
 * @param transaction
 * @return number of variables
 */
int GetNumberOfVariables(const fuse_core::Transaction::SharedPtr& transaction);

/// @brief Gets a pointer to a gryo bias variable if it exists (nullptr
/// otherwise)
/// @param graph graph to look in
/// @param stamp of variable to get
/// @return pointer to variable
bs_variables::GyroscopeBias3DStamped::SharedPtr
    GetGryoscopeBias(fuse_core::Graph::ConstSharedPtr graph,
                     const ros::Time& stamp);

/// @brief Gets a pointer to an accel bias variable if it exists (nullptr
/// otherwise)
/// @param graph graph to look in
/// @param stamp of variable to get
/// @return pointer to variable
bs_variables::AccelerationBias3DStamped::SharedPtr
    GetAccelBias(fuse_core::Graph::ConstSharedPtr graph,
                 const ros::Time& stamp);

/// @brief Gets a pointer to a position variable if it exists (nullptr
/// otherwise)
/// @param graph graph to look in
/// @param stamp of variable to get
/// @return pointer to variable
fuse_variables::Position3DStamped::SharedPtr
    GetPosition(fuse_core::Graph::ConstSharedPtr graph, const ros::Time& stamp);

/// @brief Gets a pointer to an orientation variable if it exists (nullptr
/// otherwise)
/// @param graph graph to look in
/// @param stamp of variable to get
/// @return pointer to variable
fuse_variables::Orientation3DStamped::SharedPtr
    GetOrientation(fuse_core::Graph::ConstSharedPtr graph,
                   const ros::Time& stamp);

/// @brief Gets a pointer to a velocity variable if it exists (nullptr
/// otherwise)
/// @param graph graph to look in
/// @param stamp of variable to get
/// @return pointer to variable
fuse_variables::VelocityLinear3DStamped::SharedPtr
    GetVelocity(fuse_core::Graph::ConstSharedPtr graph, const ros::Time& stamp);

/// @brief Gets a pointer to an angular velocity variable if it exists (nullptr
/// otherwise)
/// @param graph graph to look in
/// @param stamp of variable to get
/// @return pointer to variable
fuse_variables::VelocityAngular3DStamped::SharedPtr
    GetAngularVelocity(fuse_core::Graph::ConstSharedPtr graph,
                       const ros::Time& stamp);

/// @brief Gets a pointer to a linear acceleration variable if it exists
/// (nullptr otherwise)
/// @param graph graph to look in
/// @param stamp of variable to get
/// @return pointer to variable
fuse_variables::AccelerationLinear3DStamped::SharedPtr
    GetLinearAcceleration(fuse_core::Graph::ConstSharedPtr graph,
                          const ros::Time& stamp);

/// @brief Gets all timestamps in the given graph
/// @param graph to search in
/// @return set of timestamps
std::set<ros::Time> CurrentTimestamps(fuse_core::Graph::ConstSharedPtr graph);

/// @brief Gets all landmark id's in the given graph
/// @param graph to search in
/// @return set of landmark ids
std::set<uint64_t> CurrentLandmarkIDs(fuse_core::Graph::ConstSharedPtr graph);

} // namespace bs_common
