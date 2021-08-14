#pragma once

// std
#include <map>
#include <unordered_map>

// beam_slam
#include <bs_common/extrinsics_lookup.h>
#include <fuse_core/eigen.h>
#include <fuse_core/graph.h>
#include <fuse_core/macros.h>
#include <fuse_core/transaction.h>
#include <fuse_core/util.h>
#include <fuse_core/uuid.h>
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/point_3d_landmark.h>
#include <fuse_variables/position_3d_stamped.h>

// libbeam
#include <beam_calibration/CameraModel.h>
#include <beam_utils/optional.h>

namespace bs_models { namespace camera_to_camera {

class VisualMap {
public:
  /**
   * @brief Custom cosntrcutor, use when working with transactions
   * @param cam_model camera model being used
   */
  VisualMap(std::shared_ptr<beam_calibration::CameraModel> cam_model,
            const std::string& source = "VIO",
            const size_t tracked_features = 100, const size_t window_size = 20);

  /**
   * @brief Custom cosntrcutor, use when working with local graph
   * @param cam_model camera model being used
   * @param local_graph graph object being used
   */
  VisualMap(std::shared_ptr<beam_calibration::CameraModel> cam_model,
            fuse_core::Graph::SharedPtr local_graph,
            const std::string& source = "VIO",
            const size_t tracked_features = 100, const size_t window_size = 20);

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
  fuse_variables::Point3DLandmark::SharedPtr GetLandmark(uint64_t landmark_id);

  /**
   * @brief Helper function to add a pose at time t to a transaction or graph
   * The pose being added will be a camera pose, transform it to baselink
   * @param T_WORLD_CAMERA pose of camera to add to graph or transaction
   * @param cur_time timestamp of pose
   * @param transaction (optional) if provided will add to transaction,
   * otherwise will add to loca graph
   */
  void AddPose(const Eigen::Matrix4d& T_WORLD_CAMERA, const ros::Time& cur_time,
               fuse_core::Transaction::SharedPtr transaction = nullptr);

  /**
   * @brief Helper function to add a new landmark variable to a transaction or
   * graph
   * @param position of the landmark to add
   * @param lm_id of the landmark to add
   * @param transaction (optional) if provided will add to transaction,
   * otherwise will add to loca graph
   */
  void AddLandmark(const Eigen::Vector3d& position, uint64_t lm_id,
                   fuse_core::Transaction::SharedPtr transaction = nullptr);

  /**
   * @brief Helper function to add a new landmark variable to a transaction or
   * graph
   * @param landmark to add
   * @param transaction (optional) if provided will add to transaction,
   * otherwise will add to loca graph
   */
  void AddLandmark(fuse_variables::Point3DLandmark::SharedPtr landmark,
                   fuse_core::Transaction::SharedPtr transaction = nullptr);

  /**
   * @brief Helper function to add a constraint between a landmark and a pose
   * @param img_time associated image timestamp to add constraint to
   * @param landmark_id landmark to add constraint to
   * @param pixel measured pixel of landmark in image at img_time
   * @param transaction (optional) if provided will add to transaction,
   * otherwise will add to loca graph
   */
  void AddConstraint(const ros::Time& img_time, uint64_t lm_id,
                     const Eigen::Vector2d& pixel,
                     fuse_core::Transaction::SharedPtr transaction = nullptr);

  /**
   * @brief Retrieves q_WORLD_BASELINK
   * @param stamp to retrieve orientation at
   */
  fuse_variables::Orientation3DStamped::SharedPtr
      GetOrientation(const ros::Time& stamp);

  /**
   * @brief Retrieves t_WORLD_BASELINK
   * @param stamp to retrieve position at
   */
  fuse_variables::Position3DStamped::SharedPtr
      GetPosition(const ros::Time& stamp);

  /**
   * @brief Adds orientation in baselink frame
   * @param stamp associated to orientation
   * @param q_WORLD_BASELINK quaternion representing orientation
   * @param transaciton optional transaction object if using global graph
   */
  void AddOrientation(const Eigen::Quaterniond& q_WORLD_BASELINK,
                      const ros::Time& stamp,
                      fuse_core::Transaction::SharedPtr transaction = nullptr);

  /**
   * @brief Adds position in baselink frame
   * @param stamp associated to position
   * @param q_WORLD_BASELINK vector representing position
   * @param transaciton optional transaction object if using global graph
   */
  void AddPosition(const Eigen::Vector3d& p_WORLD_BASELINK,
                   const ros::Time& stamp,
                   fuse_core::Transaction::SharedPtr transaction = nullptr);

  /**
   * @brief Adds orientation in baselink frame
   * @param stamp associated to orientation
   * @param q_WORLD_BASELINK quaternion representing orientation
   * @param transaciton optional transaction object if using global graph
   */
  void AddOrientation(
      fuse_variables::Orientation3DStamped::SharedPtr orientation,
      fuse_core::Transaction::SharedPtr transaction = nullptr);

  /**
   * @brief Adds position in baselink frame
   * @param stamp associated to position
   * @param q_WORLD_BASELINK vector representing position
   * @param transaciton optional transaction object if using global graph
   */
  void AddPosition(fuse_variables::Position3DStamped::SharedPtr position,
                   fuse_core::Transaction::SharedPtr transaction = nullptr);

  /**
   * @brief Gets fuse uuid of landmark
   * @param landmark_id of landmark
   */
  fuse_core::UUID GetLandmarkUUID(uint64_t landmark_id);

  /**
   * @brief Gets fuse uuid of stamped position
   * @param stamp of position
   */
  fuse_core::UUID GetPositionUUID(ros::Time stamp);

  /**
   * @brief Gets fuse uuid of stamped orientation
   * @param stamp of orientation
   */
  fuse_core::UUID GetOrientationUUID(ros::Time stamp);

  /**
   * @brief Updates current graph copy
   * @param graph_msg graph to update with
   */
  void UpdateGraph(fuse_core::Graph::ConstSharedPtr graph_msg);

protected:
  // temp maps for in between optimization cycles
  std::map<uint64_t, fuse_variables::Orientation3DStamped::SharedPtr>
      orientations_;
  std::map<uint64_t, fuse_variables::Position3DStamped::SharedPtr> positions_;
  std::map<uint64_t, fuse_variables::Point3DLandmark::SharedPtr>
      landmark_positions_;

  // memory management variables
  size_t tracked_features_{100}; // # of features tracked per frame
  size_t window_size_{20};       // # of keyframe poses to retain in local maps

  // local graph for direct use
  fuse_core::Graph::SharedPtr local_graph_;

  // copy of the fuse graph (read only)
  fuse_core::Graph::ConstSharedPtr graph_;

  // pointer to camera model to use when adding constraints
  std::shared_ptr<beam_calibration::CameraModel> cam_model_;

  // robot extrinsics
  Eigen::Matrix4d T_cam_baselink_;
  bs_common::ExtrinsicsLookup& extrinsics_ =
      bs_common::ExtrinsicsLookup::GetInstance();

  // source for the odometry topic to use when publishing
  std::string source_{};
};

}} // namespace bs_models::camera_to_camera
