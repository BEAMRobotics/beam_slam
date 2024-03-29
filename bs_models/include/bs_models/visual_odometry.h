#pragma once

#include <mutex>
#include <queue>

#include <boost/bimap.hpp>

#include <fuse_constraints/absolute_pose_3d_stamped_constraint.h>
#include <fuse_core/async_sensor_model.h>
#include <fuse_core/macros.h>
#include <fuse_core/throttled_callback.h>

#include <beam_calibration/CameraModel.h>
#include <beam_containers/LandmarkContainer.h>
#include <beam_cv/ImageDatabase.h>
#include <beam_cv/geometry/PoseRefinement.h>

#include <bs_common/bs_msgs.h>
#include <bs_common/extrinsics_lookup_online.h>
#include <bs_models/frame_initializers/frame_initializer.h>
#include <bs_models/vision/keyframe.h>
#include <bs_models/vision/visual_map.h>
#include <bs_models/vision/vo_localization_validation.h>
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
  void onStop() override;

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
  beam::opt<Eigen::Vector3d>
      TriangulateLandmark(const uint64_t id,
                          Eigen::Vector3d& average_viewing_angle,
                          uint64_t& visual_word_id);

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
  /// @param new_graph graph pulled from on graph update
  void MarginalizeLocalGraph(const fuse_core::Graph& new_graph);

  /// @brief Updates the local graph with non visual constraints in the new
  /// graph
  /// @param new_graph graph pulled from on graph update
  void UpdateLocalGraph(const fuse_core::Graph& new_graph);

  /// @brief Publishes all landmarks in the graph as a point cloud
  /// @param graph
  void PublishLandmarkPointCloud(const fuse_core::Graph& graph);

  /// @brief Gets initial pose estimate using the frame initializer
  /// @param timestamp
  /// @param T_WORLD_BASELINK estimated pose
  /// @return whether it succeeded or not
  bool GetInitialPoseEstimate(const ros::Time& timestamp,
                              Eigen::Matrix4d& T_WORLD_BASELINK);

  /// @brief Projects the current map landmarks into an iamge and stores for
  /// search
  /// @param T_WORLD_BASELINK frame to project into
  void ProjectMapPoints(const Eigen::Matrix4d& T_WORLD_BASELINK);

  /// @brief Searches for a matching landmark using the projected local map
  /// points
  /// @param pixel input pixel measurement
  /// @param viewing_angle input viewing angle of measurement
  /// @param word_id input visual word id of measurement
  /// @param matched_id matching landmark id in the local map
  /// @return true if point matched, false otherwise
  bool SearchLocalMap(const Eigen::Vector2d& pixel,
                      const Eigen::Vector3d& viewing_angle,
                      const uint64_t word_id, uint64_t& matched_id);

  /// @brief Cleans up the new to old landmark id map
  /// @param old_ids landmark id's before marginalization
  /// @param new_ids landmark id's after marginalization
  void CleanNewToOldLandmarkMap(const std::set<uint64_t>& old_ids,
                                const std::set<uint64_t>& new_ids);

  /// @brief
  /// @param T_WORLD_BASELINK
  /// @param pixels
  /// @param points
  /// @return
  double ComputeAverageReprojection(
      const Eigen::Matrix4d& T_WORLD_BASELINK,
      const std::vector<Eigen::Vector2i, beam::AlignVec2i>& pixels,
      const std::vector<Eigen::Vector3d, beam::AlignVec3d>& points);

  /// @brief shuts down subscribers and resets to base state
  void shutdown();

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

  /// @brief subscribers/clients
  ros::Subscriber measurement_subscriber_;

  /// @brief publishers
  ros::Publisher odometry_publisher_;
  ros::Publisher keyframe_publisher_;
  ros::Publisher slam_chunk_publisher_;
  ros::Publisher imu_constraint_trigger_publisher_;
  ros::Publisher camera_landmarks_publisher_;
  ros::Publisher reset_publisher_;

  /// @brief book keeping variables
  bool is_initialized_{false};
  bool resetting_{false};
  std::map<ros::Time, vision::Keyframe> keyframes_;
  std::deque<bs_common::CameraMeasurementMsg::ConstPtr>
      visual_measurement_buffer_;
  ros::Time previous_keyframe_;
  size_t max_container_size_;
  bool track_lost_{false};
  ros::Time prev_frame_{ros::Time(0)};
  double lag_duration_;
  std::mutex buffer_mutex_;
  Eigen::Matrix4d T_WORLD_BASELINKprevframe_;
  std::shared_ptr<vision::VOLocalizationValidation> validator_;
  int num_loc_fails_in_a_row_{0};
  int imu_constraint_trigger_counter_{0};

  /// @brief local map matching stuff
  boost::bimap<uint64_t, uint64_t> new_to_old_lm_ids_;
  cv::Mat landmark_projection_mask_;
  std::map<uint64_t, uint64_t> image_projection_to_lm_id_;
  std::shared_ptr<beam_cv::ImageDatabase> image_db_;

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
  Eigen::Matrix4d T_baselink_cam_;
  bs_common::ExtrinsicsLookupOnline& extrinsics_ =
      bs_common::ExtrinsicsLookupOnline::GetInstance();

  /// @brief standalone vo stuff
  fuse_core::Graph::SharedPtr local_graph_;
  ceres::Solver::Options local_solver_options_;

  /// @brief params only changeable here
  bool use_frame_init_relative_{true};
};

} // namespace bs_models
