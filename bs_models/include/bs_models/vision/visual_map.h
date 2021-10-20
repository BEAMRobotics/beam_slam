#pragma once

#include <map>
#include <unordered_map>

#include <fuse_core/eigen.h>
#include <fuse_core/graph.h>
#include <fuse_core/macros.h>
#include <fuse_core/transaction.h>
#include <fuse_core/util.h>
#include <fuse_core/uuid.h>
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/point_3d_fixed_landmark.h>
#include <fuse_variables/point_3d_landmark.h>
#include <fuse_variables/position_3d_stamped.h>

#include <beam_calibration/CameraModel.h>
#include <beam_utils/optional.h>

#include <bs_common/extrinsics_lookup_online.h>

namespace bs_models {
namespace vision {

class VisualMap {
public:
  /**
   * @brief Custom cosntrcutor, use when working with transactions
   * @param cam_model camera model being used
   */
  VisualMap(std::shared_ptr<beam_calibration::CameraModel> cam_model,
            const std::string &source = "VIO",
            const size_t tracked_features = 100, const size_t window_size = 20);

  /**
   * @brief Default destructor
   */
  ~VisualMap() = default;

  /**
   * @brief Helper function to get T_WORLD_CAMERA at tiemstamp
   * @param stamp timestamp to get pose at
   * @return T_WORLD_CAMERA
   */
  beam::opt<Eigen::Matrix4d> GetCameraPose(const ros::Time &stamp);

  /**
   * @brief Helper function to get T_WORLD_BASELINK at tiemstamp
   * @param stamp timestamp to get pose at
   * @return T_WORLD_CAMERA
   */
  beam::opt<Eigen::Matrix4d> GetBaselinkPose(const ros::Time &stamp);

  /**
   * @brief Helper function to add a pose at time t to a transaction or graph
   * The pose being added will be a camera pose, transform it to baselink
   * @param T_WORLD_CAMERA pose of camera to add to graph or transaction
   * @param stamp timestamp of pose
   * @param transaction to add to
   */
  void AddCameraPose(const Eigen::Matrix4d &T_WORLD_CAMERA,
                     const ros::Time &stamp,
                     fuse_core::Transaction::SharedPtr transaction);
  /**
   * @brief Helper function to add a pose at time t to a transaction or graph
   * The pose being added will be a camera pose, transform it to baselink
   * @param T_WORLD_CAMERA pose of camera to add to graph or transaction
   * @param stamp timestamp of pose
   * @param transaction to add to
   */
  void AddBaselinkPose(const Eigen::Matrix4d &T_WORLD_BASELINK,
                       const ros::Time &stamp,
                       fuse_core::Transaction::SharedPtr transaction);

  /**
   * @brief Helper function to add a new landmark variable to a transaction
   * or graph
   * @param position of the landmark to add
   * @param id of the landmark to add
   * @param transaction to add to
   */
  void AddLandmark(const Eigen::Vector3d &position, uint64_t id,
                   fuse_core::Transaction::SharedPtr transaction);

  /**
   * @brief Helper function to add a new landmark variable to a transaction or
   * graph
   * @param landmark to add
   * @param transaction to add to
   */
  void AddLandmark(fuse_variables::Point3DLandmark::SharedPtr landmark,
                   fuse_core::Transaction::SharedPtr transaction);

  /**
   * @brief Helper function to add a new landmark variable to a transaction or
   * graph
   * @param position of the landmark to add
   * @param id of the landmark to add
   * @param transaction to add to
   */
  void AddFixedLandmark(const Eigen::Vector3d &position, uint64_t id,
                        fuse_core::Transaction::SharedPtr transaction);

  /**
   * @brief Helper function to add a new landmark variable to a transaction or
   * graph
   * @param landmark to add
   * @param transaction to add to
   */
  void
  AddFixedLandmark(fuse_variables::Point3DFixedLandmark::SharedPtr landmark,
                   fuse_core::Transaction::SharedPtr transaction);

  /**
   * @brief Helper function to add a constraint between a landmark and a pose
   * @param stamp associated image timestamp to add constraint to
   * @param landmark_id landmark to add constraint to
   * @param pixel measured pixel of landmark in image at img_time
   * @param transaction to add to
   */
  void AddConstraint(const ros::Time &stamp, uint64_t lm_id,
                     const Eigen::Vector2d &pixel,
                     fuse_core::Transaction::SharedPtr transaction);

  /**
   * @brief Helper function to get a landmark by id
   * @param landmark_id to retrieve
   */
  fuse_variables::Point3DLandmark::SharedPtr GetLandmark(uint64_t landmark_id);

  /**
   * @brief Helper function to get a landmark by id
   * @param landmark_id to retrieve
   */
  fuse_variables::Point3DFixedLandmark::SharedPtr
  GetFixedLandmark(uint64_t landmark_id);

  /**
   * @brief Retrieves q_WORLD_BASELINK
   * @param stamp to retrieve orientation at
   */
  fuse_variables::Orientation3DStamped::SharedPtr
  GetOrientation(const ros::Time &stamp);

  /**
   * @brief Retrieves t_WORLD_BASELINK
   * @param stamp to retrieve position at
   */
  fuse_variables::Position3DStamped::SharedPtr
  GetPosition(const ros::Time &stamp);

  /**
   * @brief Adds orientation in baselink frame
   * @param stamp associated to orientation
   * @param q_WORLD_BASELINK quaternion representing orientation
   * @param transaciton to add to
   */
  void AddOrientation(const Eigen::Quaterniond &q_WORLD_BASELINK,
                      const ros::Time &stamp,
                      fuse_core::Transaction::SharedPtr transaction);

  /**
   * @brief Adds position in baselink frame
   * @param stamp associated to position
   * @param q_WORLD_BASELINK vector representing position
   * @param transaciton to add to
   */
  void AddPosition(const Eigen::Vector3d &p_WORLD_BASELINK,
                   const ros::Time &stamp,
                   fuse_core::Transaction::SharedPtr transaction);

  /**
   * @brief Adds orientation in baselink frame
   * @param stamp associated to orientation
   * @param q_WORLD_BASELINK quaternion representing orientation
   * @param transaciton to add to
   */
  void
  AddOrientation(fuse_variables::Orientation3DStamped::SharedPtr orientation,
                 fuse_core::Transaction::SharedPtr transaction);

  /**
   * @brief Adds position in baselink frame
   * @param stamp associated to position
   * @param q_WORLD_BASELINK vector representing position
   * @param transaciton to add to
   */
  void AddPosition(fuse_variables::Position3DStamped::SharedPtr position,
                   fuse_core::Transaction::SharedPtr transaction);

  /**
   * @brief Gets fuse uuid of landmark
   * @param landmark_id of landmark
   */
  fuse_core::UUID GetLandmarkUUID(uint64_t landmark_id);

  /**
   * @brief Gets fuse uuid of stamped position
   * @param stamp of position
   */
  fuse_core::UUID GetPositionUUID(const ros::Time &stamp);

  /**
   * @brief Gets fuse uuid of stamped orientation
   * @param stamp of orientation
   */
  fuse_core::UUID GetOrientationUUID(const ros::Time &stamp);

  /**
   * @brief Checks if a given pose at a time is in the graph
   * @return bool
   */
  bool PoseExists(const ros::Time &stamp);

  /**
   * @brief Checks if a given landmark is in the graph
   * @return bool
   */
  bool LandmarkExists(uint64_t landmark_id);

  /**
   * @brief Updates current graph copy
   * @param graph_msg graph to update with
   */
  void UpdateGraph(fuse_core::Graph::SharedPtr graph_msg);

protected:
  // temp maps for in between optimization cycles
  std::map<uint64_t, fuse_variables::Orientation3DStamped::SharedPtr>
      orientations_;
  std::map<uint64_t, fuse_variables::Position3DStamped::SharedPtr> positions_;
  std::map<uint64_t, fuse_variables::Point3DLandmark::SharedPtr>
      landmark_positions_;
  std::map<uint64_t, fuse_variables::Point3DFixedLandmark::SharedPtr>
      fixed_landmark_positions_;

  // memory management variables
  size_t tracked_features_{100}; // # of features tracked per frame
  size_t window_size_{20};       // # of keyframe poses to retain in local maps

  // copy of the current graph
  fuse_core::Graph::SharedPtr graph_;

  // pointer to camera model to use when adding constraints
  std::shared_ptr<beam_calibration::CameraModel> cam_model_;

  // robot extrinsics
  Eigen::Matrix4d T_cam_baselink_;
  bs_common::ExtrinsicsLookupOnline &extrinsics_ =
      bs_common::ExtrinsicsLookupOnline::GetInstance();

  // source for the odometry topic to use when publishing
  std::string source_{};
};

} // namespace vision
} // namespace bs_models