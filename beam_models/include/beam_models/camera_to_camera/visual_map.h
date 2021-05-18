#pragma once
// std
#include <map>
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

namespace beam_models { namespace camera_to_camera {

class VisualMap {
public:
  VisualMap(std::string source,
            std::shared_ptr<beam_calibration::CameraModel> cam_model);

  ~VisualMap() = default;

  /**
   * @brief Helper function to get an orientation variable
   * @param track feature track of current image
   */
  fuse_variables::Orientation3DStamped::SharedPtr
      getOrientation(const ros::Time& stamp);

  /**
   * @brief Helper function to get a position variable
   * @param track feature track of current image
   */
  fuse_variables::Position3DStamped::SharedPtr
      getPosition(const ros::Time& stamp);

  /**
   * @brief Helper function to get a landmark by id
   * @param track feature track of current image
   */
  fuse_variables::Position3D::SharedPtr getLandmark(uint64_t landmark_id);

  /**
   * @brief Helper function to add a new orientation variable to a transaction
   * @param track feature track of current image
   */
  void addOrientation(const Eigen::Quaterniond& q, const ros::Time& cur_time,
                      std::shared_ptr<fuse_core::Transaction> transaction);

  /**
   * @brief Helper function to add a new position variable to a transaction
   * @param track feature track of current image
   */
  void addPosition(const Eigen::Vector3d& p, const ros::Time& cur_time,
                   std::shared_ptr<fuse_core::Transaction> transaction);

  /**
   * @brief Helper function to add a new landmark variable to a transaction
   * @param track feature track of current image
   */
  void addLandmark(const Eigen::Vector3d& p, uint64_t id,
                   std::shared_ptr<fuse_core::Transaction> transaction);

  /**
   * @brief Helper function to add a new landmark variable to a transaction
   * @param track feature track of current image
   */
  void addConstraint(const ros::Time& img_time, uint64_t lm_id,
                     const Eigen::Vector2d& pixel,
                     std::shared_ptr<fuse_core::Transaction> transaction);

  /**
   * @brief Updates current graph copy
   * @param track feature track of current image
   */
  void updateGraph(fuse_core::Graph::ConstSharedPtr graph_msg);

protected:
  // these store the most up to date variables for in between optimizations
  std::unordered_map<uint64_t, fuse_variables::Orientation3DStamped::SharedPtr>
      orientations_;
  std::unordered_map<uint64_t, fuse_variables::Position3DStamped::SharedPtr>
      positions_;
  std::unordered_map<uint64_t, fuse_variables::Position3D::SharedPtr>
      landmark_positions_;
  fuse_core::Graph::ConstSharedPtr graph_;
  bool graph_initialized = false;
  std::string source_;
  std::shared_ptr<beam_calibration::CameraModel> cam_model_;
};

}} // namespace beam_models::camera_to_camera
