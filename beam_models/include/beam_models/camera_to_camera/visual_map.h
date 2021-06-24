#pragma once
// std
#include <map>
#include <unordered_map>
// beam_slam
#include <beam_variables/position_3d.h>
#include <fuse_core/eigen.h>
#include <fuse_core/graph.h>
#include <fuse_core/macros.h>
#include <fuse_core/transaction.h>
#include <fuse_core/util.h>
#include <fuse_core/uuid.h>
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>
// libbeam
#include <beam_calibration/CameraModel.h>
#include <beam_utils/optional.h>

namespace beam_models { namespace camera_to_camera {

class VisualMap {
public:
  /**
   * @brief Custom cosntrcutor, use when working with transactions
   * @param cam_model camera model being used
   * @param T_imu_cam transform from camera to imu frame
   */
  VisualMap(std::shared_ptr<beam_calibration::CameraModel> cam_model,
            const Eigen::Matrix4d& T_imu_cam, const std::string& source);

  /**
   * @brief Custom cosntrcutor, use when working with local graph
   * @param cam_model camera model being used
   * @param local_graph graph object being used
   * @param T_imu_cam transform from camera to imu frame
   */
  VisualMap(std::shared_ptr<beam_calibration::CameraModel> cam_model,
            fuse_core::Graph::SharedPtr local_graph,
            const Eigen::Matrix4d& T_imu_cam, const std::string& source);

  /**
   * @brief Default destructor
   */
  ~VisualMap() = default;

  /**
   * @brief Helper function to get T_WORLD_CAMERA at
   * @param stamp timestamp to get pose at
   * @return T_WORLD_CAMERA
   */
  beam::opt<Eigen::Matrix4d> GetPose(const ros::Time& stamp);

  /**
   * @brief Helper function to get a landmark by id
   * @param landmark_id to retrieve
   */
  fuse_variables::Position3D::SharedPtr GetLandmark(uint64_t landmark_id);

  /**
   * @brief Helper function to add a pose at time t to a transaction or graph
   * The pose being added will be a camera pose, transform it to imu
   * @param T_WORLD_CAMERA pose of camera to add to graph or transaction
   * @param cur_time timestamp of pose
   * @param transaction (optional) if provided will add to transaction,
   * otherwise will add to loca graph
   */
  void AddPose(const Eigen::Matrix4d& T_WORLD_CAMERA, const ros::Time& cur_time,
               std::shared_ptr<fuse_core::Transaction> transaction = nullptr);

  /**
   * @brief Helper function to add a new landmark variable to a transaction or
   * graph
   * @param position of the landmark to add
   * @param lm_id of the landmark to add
   * @param transaction (optional) if provided will add to transaction,
   * otherwise will add to loca graph
   */
  void AddLandmark(
      const Eigen::Vector3d& position, uint64_t lm_id,
      std::shared_ptr<fuse_core::Transaction> transaction = nullptr);

  /**
   * @brief Helper function to add a constraint between a landmark and a pose
   * @param img_time associated image timestamp to add constraint to
   * @param landmark_id landmark to add constraint to
   * @param pixel measured pixel of landmark in image at img_time
   * @param transaction (optional) if provided will add to transaction,
   * otherwise will add to loca graph
   */
  void AddConstraint(
      const ros::Time& img_time, uint64_t lm_id, const Eigen::Vector2d& pixel,
      std::shared_ptr<fuse_core::Transaction> transaction = nullptr);

  /**
   * @brief Retrieves q_WORLD_IMU
   * @param stamp to retrieve orientation at
   */
  fuse_variables::Orientation3DStamped::SharedPtr
      GetOrientation(const ros::Time& stamp);

  /**
   * @brief Retrieves t_WORLD_IMU
   * @param stamp to retrieve position at
   */
  fuse_variables::Position3DStamped::SharedPtr
      GetPosition(const ros::Time& stamp);

  /**
   * @brief Updates current graph copy
   * @param graph_msg graph to update with
   */
  void UpdateGraph(fuse_core::Graph::ConstSharedPtr graph_msg);

protected:
  // these store the most up to date variables for in between optimization
  // cycles
  std::unordered_map<uint64_t, fuse_variables::Orientation3DStamped::SharedPtr>
      orientations_;
  std::unordered_map<uint64_t, fuse_variables::Position3DStamped::SharedPtr>
      positions_;
  std::unordered_map<uint64_t, fuse_variables::Position3D::SharedPtr>
      landmark_positions_;
  fuse_core::Graph::SharedPtr
      local_graph_; // local graph  if not using global fuse graph
  fuse_core::Graph::ConstSharedPtr graph_; // copy of the current fuse graph
  bool graph_initialized = false;
  std::shared_ptr<beam_calibration::CameraModel> cam_model_;
  Eigen::Matrix4d T_imu_cam_;
  std::string source_{};
};

}} // namespace beam_models::camera_to_camera
