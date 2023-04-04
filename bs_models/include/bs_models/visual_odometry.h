#pragma once

#include <queue>

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

  /**
   * @brief Default Constructor
   */
  VisualOdometry();

  /**
   * @brief Default Destructor
   */
  ~VisualOdometry() override = default;

private:
  /**
   * @brief Callback for image processing, this callback will add visual
   * constraints and triangulate new landmarks when required
   * @param[in] msg - The image to process
   */
  void processMeasurements(const CameraMeasurementMsg::ConstPtr& msg);

  /**
   * @brief Perform any required initialization for the sensor model
   *
   * This could include things like reading from the parameter server or
   * subscribing to topics. The class's node handles will be properly
   * initialized before SensorModel::onInit() is called. Spinning of the
   * callback queue will not begin until after the call to SensorModel::onInit()
   * completes.
   */
  void onInit() override;

  /**
   * @brief Subscribe to the input topics to start sending transactions to the
   * optimizer
   */
  void onStart() override;

  /**
   * @brief Unsubscribe to the input topic
   */
  void onStop() override {}

  /**
   * @brief Callback for when a newly optimized graph is available
   */
  void onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph_msg) override;

  /**
   * @brief Localizes a given frame using the tracker and the current visual map
   * @param img_time time of image to localize
   * @return T_WORLD_BASELINK
   */
  Eigen::Matrix4d LocalizeFrame(const ros::Time& img_time);

  /**
   * @brief Determines if a frame is a keyframe
   * @param img_time time of image to determine if its a keyframe
   * @param T_WORLD_BASELINK initial odometry estimate
   * @return true or false decision
   */
  bool IsKeyframe(const ros::Time& img_time,
                  const Eigen::Matrix4d& T_WORLD_BASELINK);

  /**
   * @brief Extends the map at the current keyframe time and adds the visual
   * constraints
   * @param T_WORLD_BASELINK initial odometry estimate
   */
  void ExtendMap(const Eigen::Matrix4d& T_WORLD_BASELINK,
                 const CameraMeasurementMsg::ConstPtr& msg);

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
  ros::Publisher
      odometry_publisher_; // publishes relative odometry for every frame
  ros::Publisher
      keyframe_publisher_; // publishes world odometry for every keyframe
  ros::Publisher
      slam_chunk_publisher_; // publishes a slam chunk associated to a keyframe
  ros::Publisher reloc_publisher_; // publishes a reloc request message with the
                                   // camera measurement

  size_t container_size_ = 0;
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
