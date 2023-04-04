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

  /// @brief Localizes a given frame using the tracker and the current visual
  /// map
  /// @param img_time
  /// @return pose of frame in world
  Eigen::Matrix4d LocalizeFrame(const ros::Time& img_time);

  /// @brief Determines if a frame is a keyframe
  /// @param img_time
  /// @param T_WORLD_BASELINK
  /// @return whether a frame is a keyframe
  bool IsKeyframe(const ros::Time& img_time,
                  const Eigen::Matrix4d& T_WORLD_BASELINK);

  /// @brief Extends the map at the current keyframe time and adds the visual
  /// constraints
  /// @param T_WORLD_BASELINK
  void ExtendMap(const Eigen::Matrix4d& T_WORLD_BASELINK);

  /// @brief Adds visual measurements to the landmark container
  /// @param msg
  void AddMeasurementsToContainer(const CameraMeasurementMsg::ConstPtr& msg);

  /// @brief Gets 2d-3d correspondences for landmarks measured at a given time
  /// @param img_time
  /// @param pixels
  /// @param points
  void GetPixelPointPairs(
      const ros::Time& img_time,
      std::vector<Eigen::Vector2i, beam::AlignVec2i>& pixels,
      std::vector<Eigen::Vector3d, beam::AlignVec3d>& points);

  /// @brief Creates a prior constraint given a pose and covariance
  /// @param position
  /// @param orientation
  /// @param covariance
  /// @return Shared pointer to constraint
  std::shared_ptr<fuse_constraints::AbsolutePose3DStampedConstraint>
      MakeFrameInitPrior(
          const fuse_variables::Position3DStamped& position,
          const fuse_variables::Orientation3DStamped& orientation,
          const Eigen::Matrix<double, 6, 6>& covariance);

  fuse_core::UUID device_id_; //!< The UUID of this device
  // loadable camera parameters
  bs_parameters::models::VisualOdometryParams vo_params_;

  // calibration parameters
  bs_parameters::models::CalibrationParams calibration_params_;

  // Used to get initial pose estimates
  std::unique_ptr<frame_initializers::FrameInitializerBase> frame_initializer_;

  // subscribers
  ros::Subscriber measurement_subscriber_;

  // publishers
  ros::Publisher odometry_publisher_;
  ros::Publisher keyframe_publisher_;
  ros::Publisher slam_chunk_publisher_;
  ros::Publisher reloc_publisher_;

  bool is_initialized_{false};
  std::deque<Keyframe> keyframes_;
  uint32_t added_since_kf_{0};

  // callbacks for messages
  using ThrottledMeasurementCallback =
      fuse_core::ThrottledMessageCallback<CameraMeasurementMsg>;
  ThrottledMeasurementCallback throttled_measurement_callback_;

  // computer vision objects
  std::shared_ptr<beam_calibration::CameraModel> cam_model_;
  std::shared_ptr<beam_containers::LandmarkContainer> landmark_container_;
  std::shared_ptr<VisualMap> visual_map_;
  std::shared_ptr<beam_cv::PoseRefinement> pose_refiner_;

  // robot extrinsics
  Eigen::Matrix4d T_cam_baselink_;
  bs_common::ExtrinsicsLookupOnline& extrinsics_ =
      bs_common::ExtrinsicsLookupOnline::GetInstance();
};

} // namespace bs_models
