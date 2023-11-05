#pragma once

#include <mutex>
#include <queue>

#include <fuse_constraints/absolute_pose_3d_stamped_constraint.h>
#include <fuse_core/async_sensor_model.h>
#include <fuse_core/macros.h>
#include <fuse_core/throttled_callback.h>

#include <beam_calibration/CameraModel.h>
#include <beam_containers/LandmarkContainer.h>
#include <beam_cv/geometry/PoseRefinement.h>

#include <bs_common/bs_msgs.h>
#include <bs_common/extrinsics_lookup_online.h>
#include <bs_models/frame_initializers/frame_initializer.h>
#include <bs_models/vision/keyframe.h>
#include <bs_models/vision/visual_map.h>
#include <bs_parameters/models/calibration_params.h>
#include <bs_parameters/models/visual_odometry_params.h>

namespace bs_models {

class VisualOdometry : public fuse_core::AsyncSensorModel {
public:
  SMART_PTR_DEFINITIONS(VisualOdometry);

  /// @brief Default Constructor
  VisualOdometry();

  /// @brief Default Destructor
  ~VisualOdometry() override = default;

private:
  /// @brief Callback for image processing, this callback will add visual
  /// constraints and triangulate new landmarks when required
  /// @param msg The visual measurements to process
  void
      processMeasurements(const bs_common::CameraMeasurementMsg::ConstPtr& msg);

  /// @brief Perform any required initialization for the sensor model
  /// This could include things like reading from the parameter server or
  /// subscribing to topics. The class's node handles will be
  /// properlyinitialized before SensorModel::onInit() is called. Spinning of
  /// thecallback queue will not begin until after the call to
  /// SensorModel::onInit() completes.
  void onInit() override;

  /// @brief Subscribe to the input topics to start sending transactions to the
  /// optimizer
  void onStart() override;

  /// @brief Unsubscribe to the input topics and clear memory
  void onStop() override {}

  /// @brief Callback for when a newly optimized graph is available
  /// @param graph_msg incoming grpah
  void onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph) override;

  /// @brief Localizes image and extends our visual map if its a keyframe
  /// @param msg visual measurements
  /// @return whether it succeeded in localizing
  bool ComputeOdometryAndExtendMap(
      const bs_common::CameraMeasurementMsg::ConstPtr& msg);

  /// @brief Localizes a given frame using the tracker and the current visual
  /// map
  /// @param timestamp
  /// @param T_WORLD_BASELINK estimated pose
  /// @param covariance
  /// @return whether it succeeded or not
  bool LocalizeFrame(const ros::Time& timestamp,
                     Eigen::Matrix4d& T_WORLD_BASELINK,
                     Eigen::Matrix<double, 6, 6>& covariance);

  /// @brief Extends the map at the current keyframe time and adds the visual
  /// constraints
  /// @param timestamp
  /// @param T_WORLD_BASELINK
  /// @param covariance
  void ExtendMap(const ros::Time& timestamp,
                 const Eigen::Matrix4d& T_WORLD_BASELINK,
                 const Eigen::Matrix<double, 6, 6>& covariance);

  /// @brief Determines if a frame is a keyframe
  /// @param timestamp
  /// @param T_WORLD_BASELINK
  /// @return whether a frame is a keyframe
  bool IsKeyframe(const ros::Time& timestamp,
                  const Eigen::Matrix4d& T_WORLD_BASELINK);

  /// @brief Adds visual measurements to the landmark container
  /// @param msg
  void AddMeasurementsToContainer(
      const bs_common::CameraMeasurementMsg::ConstPtr& msg);

  /// @brief Triangulates a landmark of the given id
  /// @param id of landmark
  /// @return optional 3d location
  beam::opt<Eigen::Vector3d> TriangulateLandmark(const uint64_t id);

  /// @brief Gets 2d-3d correspondences for landmarks measured at a given time
  /// @param timestamp
  /// @param pixels
  /// @param points
  void GetPixelPointPairs(
      const ros::Time& timestamp,
      std::vector<Eigen::Vector2i, beam::AlignVec2i>& pixels,
      std::vector<Eigen::Vector3d, beam::AlignVec3d>& points);

  /// @brief
  /// @param timestamp
  /// @param T_WORLD_BASELINK
  /// @param covariance
  void PublishOdometry(const ros::Time& timestamp,
                       const Eigen::Matrix4d& T_WORLD_BASELINK,
                       const Eigen::Matrix<double, 6, 6>& covariance);

  /// @brief Publishes a keyframe object as a slam chunk
  /// @param keyframe
  void PublishSlamChunk(const vision::Keyframe& keyframe);

  /// @brief
  /// @param timestamp
  /// @param T_WORLD_BASELINK
  void PublishPose(const ros::Time& timestamp,
                   const Eigen::Matrix4d& T_WORLD_BASELINK);

  /// @brief Performs all initial setup after slam initialization succeeds
  /// @param graph initial graph
  void Initialize(fuse_core::Graph::ConstSharedPtr graph);

  /// @brief Add all required variables and constraints for a specific landmark
  /// using the IDP parameterization
  /// @param id of landmark to add
  /// @param timestamp timestamp of measurement
  /// @param transaction transaction to ammed to
  void ProcessLandmarkIDP(const uint64_t id, const ros::Time& timestamp,
                          fuse_core::Transaction::SharedPtr transaction);

  /// @brief Add all required variables and constraints for a specific landmark
  /// using the euclidean parameterization
  /// @param id of landmark to add
  /// @param timestamp timestamp of measurement
  /// @param transaction transaction to ammed to
  void ProcessLandmarkEUC(const uint64_t id, const ros::Time& timestamp,
                          fuse_core::Transaction::SharedPtr transaction);

  /// @brief Creates a visual odometry factor between this frame and th previous
  /// keyframe
  /// @param timestamp_curframe
  /// @param T_WORLD_BASELINKcurframe
  /// @param covariance
  /// @return
  fuse_core::Transaction::SharedPtr CreateVisualOdometryFactor(
      const ros::Time& timestamp_curframe,
      const Eigen::Matrix4d& T_WORLD_BASELINKcurframe,
      const Eigen::Matrix<double, 6, 6>& covariance);

  /// @brief Prunes the keyframes using the new graph and publishes them as
  /// slam chunks
  /// @param new_graph the newly updated graph (from ongraphupdate or in our own
  /// marginalization)
  void PruneKeyframes(const fuse_core::Graph& new_graph);

  /// @brief Marginalizes the current local graph is standalone vo is being used
  void MarginalizeGraph();

  /// @brief Computes the pose as a 7d vector (x,y,z, qw,
  /// qx, qy, qz)
  /// @param T_A_B
  /// @return delta from B to A
  fuse_core::Vector7d ComputeDelta(const Eigen::Matrix4d& T_A_B);

  /// @brief Publishes all landmarks in the graph as a point cloud
  /// @param graph
  void PublishLandmarkPointCloud(const fuse_core::Graph& graph);

  /******************************************************
   *                   Member Variables                 *
   *****************************************************/
  /// @brief The UUID of this device
  fuse_core::UUID device_id_; //!< The UUID of this device

  /// @brief loadable camera parameters
  bs_parameters::models::VisualOdometryParams vo_params_;

  /// @brief calibration parameters
  bs_parameters::models::CalibrationParams calibration_params_;

  /// @brief Used to get initial pose estimates
  std::unique_ptr<bs_models::FrameInitializer> frame_initializer_;

  /// @brief subscribers
  ros::Subscriber measurement_subscriber_;
  /// @brief publishers
  ros::Publisher odometry_publisher_;
  ros::Publisher keyframe_publisher_;
  ros::Publisher slam_chunk_publisher_;
  ros::Publisher imu_constraint_trigger_publisher_;
  ros::Publisher camera_landmarks_publisher_;
  int imu_constraint_trigger_counter_{0};

  /// @brief book keeping variables
  bool is_initialized_{false};
  std::map<ros::Time, vision::Keyframe> keyframes_;
  std::deque<bs_common::CameraMeasurementMsg::ConstPtr>
      visual_measurement_buffer_;
  ros::Time previous_reloc_request_{ros::Time(0)};
  ros::Time previous_keyframe_;
  size_t max_container_size_;
  bool track_lost{false};
  ros::Time prev_frame_{ros::Time(0)};
  double lag_duration_;
  std::mutex buffer_mutex_;

  /// @brief callbacks for messages
  using ThrottledMeasurementCallback =
      fuse_core::ThrottledMessageCallback<bs_common::CameraMeasurementMsg>;
  ThrottledMeasurementCallback throttled_measurement_callback_;

  /// @brief computer vision objects
  std::shared_ptr<beam_calibration::CameraModel> cam_model_;
  Eigen::Matrix3d cam_intrinsic_matrix_;
  cv::Mat K_;
  std::shared_ptr<beam_containers::LandmarkContainer> landmark_container_;
  std::shared_ptr<vision::VisualMap> visual_map_;
  std::shared_ptr<beam_cv::PoseRefinement> pose_refiner_;

  /// @brief robot extrinsics
  Eigen::Matrix4d T_cam_baselink_;
  bs_common::ExtrinsicsLookupOnline& extrinsics_ =
      bs_common::ExtrinsicsLookupOnline::GetInstance();

  /// @brief standalone vo stuff
  fuse_core::Vector7d keyframe_imu_delta_;
  fuse_core::Graph::SharedPtr local_graph_;
  ceres::Solver::Options local_solver_options_;
  Eigen::Matrix<double, 6, 6> imu_covariance_;
};

} // namespace bs_models
