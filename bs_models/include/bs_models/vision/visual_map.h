#pragma once

#include <map>
#include <unordered_map>

#include <bs_variables/inverse_depth_landmark.h>
#include <bs_variables/orientation_3d.h>
#include <bs_variables/point_3d_landmark.h>
#include <bs_variables/position_3d.h>
#include <fuse_core/eigen.h>
#include <fuse_core/fuse_macros.h>
#include <fuse_core/graph.h>
#include <fuse_core/transaction.h>
#include <fuse_core/util.h>
#include <fuse_core/uuid.h>
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/point_3d_fixed_landmark.h>
#include <fuse_variables/position_3d_stamped.h>

#include <beam_calibration/CameraModel.h>
#include <beam_utils/optional.h>

#include <bs_common/extrinsics_lookup_online.h>

namespace bs_models { namespace vision {

class VisualMap {
public:
  /**
   * @brief Custom constructor
   * @param cam_model camera model being used
   */
  VisualMap(const std::string& source,
            std::shared_ptr<beam_calibration::CameraModel> cam_model,
            fuse_core::Loss::SharedPtr loss_function,
            const double reprojection_information_weight,
            const bool use_online_calibration = false,
            const bool add_calibration_prior = false);

  /**
   * @brief Default destructor
   */
  ~VisualMap() = default;

  /**
   * @brief Helper function to get T_WORLD_CAMERA at tiemstamp
   * @param stamp timestamp to get pose at
   * @return T_WORLD_CAMERA
   */
  beam::opt<Eigen::Matrix4d> GetCameraPose(const ros::Time& stamp);

  /**
   * @brief Helper function to get T_WORLD_BASELINK at tiemstamp
   * @param stamp timestamp to get pose at
   * @return T_WORLD_BASELINK
   */
  beam::opt<Eigen::Matrix4d> GetBaselinkPose(const ros::Time& stamp);

  /**
   * @brief Helper function to add a pose at time t to a transaction or graph
   * @param T_WORLD_CAMERA pose of camera to add to graph or transaction
   * @param stamp timestamp of pose
   * @param transaction to add to
   */
  void AddCameraPose(const Eigen::Matrix4d& T_WORLD_CAMERA,
                     const ros::Time& stamp,
                     fuse_core::Transaction::SharedPtr transaction);
  /**
   * @brief Helper function to add a pose at time t to a transaction or graph
   * @param T_WORLD_BASELINK pose of baselink to add to graph or transaction
   * @param stamp timestamp of pose
   * @param transaction to add to
   */
  void AddBaselinkPose(const Eigen::Matrix4d& T_WORLD_BASELINK,
                       const ros::Time& stamp,
                       fuse_core::Transaction::SharedPtr transaction);

  /**
   * @brief Helper function to add a new landmark variable to a transaction
   * or graph
   * @param position of the landmark to add
   * @param id of the landmark to add
   * @param transaction to add to
   */
  void AddLandmark(const Eigen::Vector3d& position,
                   const Eigen::Vector3d& viewing_angle, const uint64_t word_id,
                   const uint64_t id,
                   fuse_core::Transaction::SharedPtr transaction);

  /**
   * @brief Helper function to add a new landmark variable to a transaction or
   * graph
   * @param landmark to add
   * @param transaction to add to
   */
  void AddLandmark(bs_variables::Point3DLandmark::SharedPtr landmark,
                   fuse_core::Transaction::SharedPtr transaction);

  /**
   * @brief Helper function to add a constraint between a landmark and a pose
   * @param stamp associated image timestamp to add constraint to
   * @param landmark_id landmark to add constraint to
   * @param pixel measured pixel of landmark in image at img_time
   * @param transaction to add to
   */
  bool AddVisualConstraint(const ros::Time& stamp, uint64_t lm_id,
                           const Eigen::Vector2d& pixel,
                           fuse_core::Transaction::SharedPtr transaction);
  /**
   * @brief Helper function to add a new inverse depth landmark variable to a
   * transaction or graph
   * @param bearing
   * @param inverse_depth
   * @param id
   * @param anchor_time
   */
  void AddInverseDepthLandmark(const Eigen::Vector3d& bearing,
                               const double inverse_depth, const uint64_t id,
                               const ros::Time& anchor_time,
                               fuse_core::Transaction::SharedPtr transaction);

  /**
   * @brief Helper function to add a new landmark variable to a transaction or
   * graph
   * @param landmark to add
   * @param transaction to add to
   */
  void AddInverseDepthLandmark(
      bs_variables::InverseDepthLandmark::SharedPtr landmark,
      fuse_core::Transaction::SharedPtr transaction);

  /**
   * @brief Helper function to add a constraint between a landmark and a pose
   * @param stamp associated image timestamp to add constraint to
   * @param landmark_id landmark to add constraint to
   * @param pixel measured pixel of landmark in image at img_time
   * @param transaction to add to
   */
  bool AddInverseDepthVisualConstraint(
      const ros::Time& measurement_stamp, uint64_t lm_id,
      const Eigen::Vector2d& pixel,
      fuse_core::Transaction::SharedPtr transaction);

  /**
   * @brief Helper function to get a landmark by id
   * @param landmark_id to retrieve
   */
  bs_variables::Point3DLandmark::SharedPtr GetLandmark(uint64_t landmark_id);

  /**
   * @brief Helper function to get a landmark by id
   * @param landmark_id to retrieve
   */
  bs_variables::InverseDepthLandmark::SharedPtr
      GetInverseDepthLandmark(uint64_t landmark_id);

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
   * @brief Adds an asbolute pose prior
   * @param stamp to add prior on
   */
  void AddPosePrior(const ros::Time& stamp,
                    const Eigen::Matrix<double, 6, 6>& covariance,
                    fuse_core::Transaction::SharedPtr transaction);

  /**
   * @brief Adds a relative pose constraint between two poses
   */
  void AddRelativePoseConstraint(const ros::Time& stamp1,
                                 const ros::Time& stamp2,
                                 const fuse_core::Vector7d& delta,
                                 const Eigen::Matrix<double, 6, 6>& covariance,
                                 fuse_core::Transaction::SharedPtr transaction);

  /**
   * @brief Adds a relative pose constraint between two poses
   */
  void AddRelativePoseConstraintOnlineCalib(
      const ros::Time& stamp1, const ros::Time& stamp2,
      const fuse_core::Vector7d& delta,
      const Eigen::Matrix<double, 6, 6>& covariance,
      fuse_core::Transaction::SharedPtr transaction);

  /**
   * @brief Adds orientation in baselink frame
   * @param stamp associated to orientation
   * @param q_WORLD_BASELINK quaternion representing orientation
   * @param transaciton to add to
   */
  void AddOrientation(const Eigen::Quaterniond& q_WORLD_BASELINK,
                      const ros::Time& stamp,
                      fuse_core::Transaction::SharedPtr transaction);

  /**
   * @brief Adds position in baselink frame
   * @param stamp associated to position
   * @param p_WORLD_BASELINK vector representing position
   * @param transaciton to add to
   */
  void AddPosition(const Eigen::Vector3d& p_WORLD_BASELINK,
                   const ros::Time& stamp,
                   fuse_core::Transaction::SharedPtr transaction);

  /**
   * @brief Adds orientation in baselink frame
   * @param orientation variable
   * @param transaciton to add to
   */
  void AddOrientation(
      fuse_variables::Orientation3DStamped::SharedPtr orientation,
      fuse_core::Transaction::SharedPtr transaction);

  /**
   * @brief Adds position in baselink frame
   * @param position variable
   * @param transaciton to add to
   */
  void AddPosition(fuse_variables::Position3DStamped::SharedPtr position,
                   fuse_core::Transaction::SharedPtr transaction);

  /**
   * @brief Adds camera extrinsic variable to the transaction
   * @param transaction to add to
   */
  void AddCameraCalibration(fuse_core::Transaction::SharedPtr transaction);

  /**
   * @brief Gets all landmark ids
   * @return set of ids
   */
  std::set<uint64_t> GetLandmarkIDs();

  /**
   * @brief Gets all landmarks
   * @return map of landmarks <id: point>
   */
  std::map<uint64_t, Eigen::Vector3d> GetLandmarks();

  /**
   * @brief Gets fuse uuid of landmark
   * @param landmark_id of landmark
   */
  fuse_core::UUID GetLandmarkUUID(uint64_t landmark_id);

  /**
   * @brief Gets fuse uuid of stamped position
   * @param stamp of position
   */
  fuse_core::UUID GetPositionUUID(const ros::Time& stamp);

  /**
   * @brief Gets fuse uuid of stamped orientation
   * @param stamp of orientation
   */
  fuse_core::UUID GetOrientationUUID(const ros::Time& stamp);

  /**
   * @brief Checks if a given pose at a time is in the graph
   * @return bool
   */
  bool PoseExists(const ros::Time& stamp);

  /**
   * @brief Checks if a given landmark is in the graph
   * @return bool
   */
  bool LandmarkExists(uint64_t landmark_id);

  /**
   * @brief Checks if a given landmark is in the graph
   * @return bool
   */
  bool FixedLandmarkExists(uint64_t landmark_id);

  /**
   * @brief Updates current graph copy
   * @param graph_msg graph to update with
   */
  void UpdateGraph(const fuse_core::Graph& graph_msg);

  /**
   * @brief Resets map to empty state
   */
  void Clear();

  /**
   * @brief Returns the set of all timestamps in the graph
   */
  std::set<ros::Time> CurrentTimestamps();

protected:
  std::string source_;
  // temp maps for in between optimization cycles
  std::map<uint64_t, fuse_variables::Orientation3DStamped::SharedPtr>
      orientations_;
  std::map<uint64_t, fuse_variables::Position3DStamped::SharedPtr> positions_;
  std::map<uint64_t, bs_variables::Point3DLandmark::SharedPtr>
      landmark_positions_;
  std::map<uint64_t, bs_variables::InverseDepthLandmark::SharedPtr>
      inversedepth_landmark_positions_;
  bs_variables::Position3D::SharedPtr p_BASELINK_CAM_;
  bs_variables::Orientation3D::SharedPtr o_BASELINK_CAM_;

  // copy of the current graph
  fuse_core::Graph::SharedPtr graph_;

  // pointer to camera model to use when adding constraints
  std::shared_ptr<beam_calibration::CameraModel> cam_model_;
  fuse_core::Loss::SharedPtr loss_function_;
  Eigen::Matrix3d camera_intrinsic_matrix_;
  double reprojection_information_weight_;

  // robot extrinsics
  Eigen::Matrix4d T_cam_baselink_;
  Eigen::Matrix4d T_baselink_cam_;
  bs_common::ExtrinsicsLookupOnline& extrinsics_ =
      bs_common::ExtrinsicsLookupOnline::GetInstance();

  // online calib stuff
  bool use_online_calibration_{false};
  bool calibration_added_{false};
  bool add_calibration_prior_{false};
};

}} // namespace bs_models::vision