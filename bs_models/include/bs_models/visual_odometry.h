#pragma once

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
#include <bs_models/frame_initializers/frame_initializers.h>
#include <bs_models/vision/keyframe.h>
#include <bs_models/vision/visual_map.h>
#include <bs_parameters/models/calibration_params.h>
#include <bs_parameters/models/visual_odometry_params.h>

namespace bs_models {

using namespace bs_common;
using namespace vision;

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
  void processMeasurements(const CameraMeasurementMsg::ConstPtr& msg);

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
  void onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph_msg) override;

  /// @brief Localizes image and extends our visual map if its a keyframe
  /// @param msg visual measurements
  /// @return whether it succeeded in localizing
  bool ComputeOdometryAndExtendMap(const CameraMeasurementMsg::ConstPtr& msg);

  /// @brief Localizes a given frame using the tracker and the current visual
  /// map
  /// @param timestamp
  /// @param T_WORLD_BASELINK estimated pose
  /// @return whether it succeeded or not
  bool LocalizeFrame(const ros::Time& timestamp,
                     Eigen::Matrix4d& T_WORLD_BASELINK);

  /// @brief Extends the map at the current keyframe time and adds the visual
  /// constraints
  /// @param timestamp
  /// @param T_WORLD_BASELINK
  void ExtendMap(const ros::Time& timestamp,
                 const Eigen::Matrix4d& T_WORLD_BASELINK);

  /// @brief Determines if a frame is a keyframe
  /// @param timestamp
  /// @param T_WORLD_BASELINK
  /// @return whether a frame is a keyframe
  bool IsKeyframe(const ros::Time& timestamp,
                  const Eigen::Matrix4d& T_WORLD_BASELINK);

  /// @brief Adds visual measurements to the landmark container
  /// @param msg
  void AddMeasurementsToContainer(const CameraMeasurementMsg::ConstPtr& msg);

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
  void ComputeRelativeOdometry(const ros::Time& timestamp,
                               const Eigen::Matrix4d& T_WORLD_BASELINKcur);

  /// @brief Publishes a keyframe object as a slam chunk
  /// @param keyframe
  void PublishSlamChunk(const Keyframe& keyframe);

  /// @brief Publishes a keyframe object as a reloc request
  /// @param keyframe
  void PublishRelocRequest(const Keyframe& keyframe);

  /// @brief
  /// @param timestamp
  /// @param T_WORLD_BASELINK
  void PublishPose(const ros::Time& timestamp,
                   const Eigen::Matrix4d& T_WORLD_BASELINK);

  // /// @brief
  // /// @param stamp
  // /// @param T_WORLD_BASELINK
  // void AddLocalCameraPose(const ros::Time& stamp,
  //                         const Eigen::Matrix4d& T_WORLD_BASELINK);

  // /// @brief
  // /// @param stamp
  // /// @param id
  // /// @param pixel
  // bool AddLocalVisualConstraint(const ros::Time& stamp, const uint64_t id,
  //                               const Eigen::Vector2d& pixel);

  // /// @brief
  // /// @param stamp
  // /// @param covariance
  // void AddLocalPosePrior(const ros::Time& stamp,
  //                        const Eigen::Matrix<double, 6, 6>& covariance);

  // /// @brief
  // /// @param stamp
  // beam::opt<Eigen::Matrix4d> GetLocalBaselinkPose(const ros::Time& stamp);

  /// @brief The UUID of this device
  fuse_core::UUID device_id_; //!< The UUID of this device
  /// @brief loadable camera parameters
  bs_parameters::models::VisualOdometryParams vo_params_;
  /// @brief calibration parameters
  bs_parameters::models::CalibrationParams calibration_params_;
  /// @brief Used to get initial pose estimates
  std::unique_ptr<frame_initializers::FrameInitializerBase> frame_initializer_;
  /// @brief subscribers
  ros::Subscriber measurement_subscriber_;
  /// @brief publishers
  ros::Publisher odometry_publisher_;
  ros::Publisher keyframe_publisher_;
  ros::Publisher slam_chunk_publisher_;
  ros::Publisher reloc_publisher_;
  /// @brief book keeping variables
  bool is_initialized_{false};
  std::deque<Keyframe> keyframes_;
  uint32_t added_since_kf_{0};
  std::deque<CameraMeasurementMsg::ConstPtr> visual_measurement_buffer_;
  Eigen::Matrix4d T_ODOM_BASELINKprev_{Eigen::Matrix4d::Identity()};
  ros::Time previous_reloc_request_{ros::Time(0)};
  ros::Time previous_frame_;
  /// @brief callbacks for messages
  using ThrottledMeasurementCallback =
      fuse_core::ThrottledMessageCallback<CameraMeasurementMsg>;
  ThrottledMeasurementCallback throttled_measurement_callback_;
  /// @brief computer vision objects
  std::shared_ptr<beam_calibration::CameraModel> cam_model_;
  Eigen::Matrix3d cam_intrinsic_matrix_;
  std::shared_ptr<beam_containers::LandmarkContainer> landmark_container_;
  std::shared_ptr<VisualMap> visual_map_;
  std::shared_ptr<beam_cv::PoseRefinement> pose_refiner_;
  fuse_core::Graph::UniquePtr local_graph_;
  /// @brief robot extrinsics
  Eigen::Matrix4d T_cam_baselink_;
  bs_common::ExtrinsicsLookupOnline& extrinsics_ =
      bs_common::ExtrinsicsLookupOnline::GetInstance();
};

} // namespace bs_models
