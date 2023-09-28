#pragma once

#include <bs_variables/accel_bias_3d_stamped.h>
#include <bs_variables/gyro_bias_3d_stamped.h>
#include <bs_variables/inverse_depth_landmark.h>
#include <bs_variables/orientation_3d.h>
#include <bs_variables/point_3d_landmark.h>
#include <bs_variables/position_3d.h>
#include <fuse_core/graph.h>
#include <fuse_core/transaction.h>
#include <fuse_core/variable.h>
#include <fuse_variables/acceleration_linear_3d_stamped.h>
#include <fuse_variables/velocity_angular_3d_stamped.h>

#include <beam_utils/utils.h>
#include <bs_common/imu_state.h>

namespace bs_common {

struct ImuBiases {
  double a_x;
  double a_y;
  double a_z;
  double g_x;
  double g_y;
  double g_z;
};

std::map<int64_t, ImuBiases>
    GetImuBiasesFromGraph(const fuse_core::Graph& graph);

/**
 * @brief Save graph to text file using the print function
 */
void SaveGraphToTxtFile(fuse_core::Graph::ConstSharedPtr graph,
                        const std::string& txt_file_save_path);

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

/**
 * @brief Get number of constraints in a graph message
 * @param graph
 * @return number of constraints
 */
int GetNumberOfConstraints(const fuse_core::Graph& graph);

/**
 * @brief Get number of variables in a graph message
 * @param graph
 * @return number of variables
 */
int GetNumberOfVariables(const fuse_core::Graph& graph);

/**
 * @brief Gets a pointer to a gryo bias variable if it exists (nullptr
 * otherwise)
 * @param graph graph to look in
 * @param stamp of variable to get
 * @return pointer to variable
 */
bs_variables::GyroscopeBias3DStamped::SharedPtr
    GetGyroscopeBias(fuse_core::Graph::ConstSharedPtr graph,
                     const ros::Time& stamp);

/**
 * @brief Gets a pointer to an accel bias variable if it exists (nullptr
 * otherwise)
 * @param graph graph to look in
 * @param stamp of variable to get
 * @return pointer to variable
 */
bs_variables::AccelerationBias3DStamped::SharedPtr
    GetAccelBias(fuse_core::Graph::ConstSharedPtr graph,
                 const ros::Time& stamp);

/**
 * @brief Gets a pointer to a position variable if it exists (nullptr
 * otherwise)
 * @param graph graph to look in
 * @param stamp of variable to get
 * @return pointer to variable
 */
fuse_variables::Position3DStamped::SharedPtr
    GetPosition(fuse_core::Graph::ConstSharedPtr graph, const ros::Time& stamp);

/**
 * @brief Gets a pointer to an orientation variable if it exists (nullptr
 * otherwise)
 * @param graph graph to look in
 * @param stamp of variable to get
 * @return pointer to variable
 */
fuse_variables::Orientation3DStamped::SharedPtr
    GetOrientation(fuse_core::Graph::ConstSharedPtr graph,
                   const ros::Time& stamp);

/**
 * @brief Gets a pointer to a velocity variable if it exists (nullptr
 * otherwise)
 * @param graph graph to look in
 * @param stamp of variable to get
 * @return pointer to variable
 */
fuse_variables::VelocityLinear3DStamped::SharedPtr
    GetVelocity(fuse_core::Graph::ConstSharedPtr graph, const ros::Time& stamp);

/**
 * @brief Gets a pointer to an angular velocity variable if it exists (nullptr
 * otherwise)
 * @param graph graph to look in
 * @param stamp of variable to get
 * @return pointer to variable
 */
fuse_variables::VelocityAngular3DStamped::SharedPtr
    GetAngularVelocity(fuse_core::Graph::ConstSharedPtr graph,
                       const ros::Time& stamp);

/**
 * @brief Gets a pointer to a linear acceleration variable if it exists
 * (nullptr otherwise)
 * @param graph graph to look in
 * @param stamp of variable to get
 * @return pointer to variable
 */
fuse_variables::AccelerationLinear3DStamped::SharedPtr
    GetLinearAcceleration(fuse_core::Graph::ConstSharedPtr graph,
                          const ros::Time& stamp);

/**
 * @brief Gets all timestamps in the given graph
 * @param graph to search in
 * @return set of timestamps
 */
std::set<ros::Time> CurrentTimestamps(fuse_core::Graph::ConstSharedPtr graph);

/**
 * @brief Gets all landmark id's in the given graph
 * @param graph to search in
 * @return set of landmark ids
 */
std::set<uint64_t> CurrentLandmarkIDs(fuse_core::Graph::ConstSharedPtr graph);

/// @brief Gets landmark variable from graph
/// @param graph to search in
/// @return set of landmark ids
bs_variables::Point3DLandmark::SharedPtr
    GetLandmark(fuse_core::Graph::ConstSharedPtr graph, const uint64_t id);

/// @brief Gets position extrinsic from graph
/// @param graph to search in
bs_variables::Position3D::SharedPtr
    GetPositionExtrinsic(fuse_core::Graph::ConstSharedPtr graph,
                         const std::string& child_frame,
                         const std::string& parent_frame);

/// @brief Gets orientation extrinsic from graph
/// @param graph to search in
bs_variables::Orientation3D::SharedPtr
    GetOrientationExtrinsic(fuse_core::Graph::ConstSharedPtr graph,
                            const std::string& child_frame,
                            const std::string& parent_frame);

/// @brief Gets extrinsic as 4x4 matrix from graph
/// @param graph to search in
beam::opt<Eigen::Matrix4d> GetExtrinsic(fuse_core::Graph::ConstSharedPtr graph,
                                        const std::string& child_frame,
                                        const std::string& parent_frame);

/// @brief Gets inverse depth landmark variable from graph
/// @param graph to search in
/// @return set of landmark ids
bs_variables::InverseDepthLandmark::SharedPtr
    GetInverseDepthLandmark(fuse_core::Graph::ConstSharedPtr graph,
                            const uint64_t id);
/// @brief
/// @param graph
/// @param stamp
/// @return
beam::opt<bs_common::ImuState>
    GetImuState(fuse_core::Graph::ConstSharedPtr graph, const ros::Time& stamp);
} // namespace bs_common
