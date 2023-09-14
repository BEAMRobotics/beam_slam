#pragma once

#include <queue>

#include <fuse_core/async_sensor_model.h>
#include <fuse_core/fuse_macros.h>
#include <fuse_core/throttled_callback.h>
#include <sensor_msgs/Image.h>

#include <beam_cv/trackers/Trackers.h>

#include <bs_common/bs_msgs.h>
#include <bs_common/extrinsics_lookup_online.h>
#include <bs_parameters/models/visual_feature_tracker_params.h>

namespace bs_models {

class VisualFeatureTracker : public fuse_core::AsyncSensorModel {
public:
  FUSE_SMART_PTR_DEFINITIONS(VisualFeatureTracker);

  /**
   * @brief Default Constructor
   */
  VisualFeatureTracker();

  /**
   * @brief Default Destructor
   */
  ~VisualFeatureTracker() override = default;

private:
  /**
   * @brief Callback for image processing, this callback will add visual
   * constraints and triangulate new landmarks when required
   * @param[in] msg - The image to process
   */
  void processImage(const sensor_msgs::Image::ConstPtr& msg);

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
   * @brief Builds a camera measurement message for an image
   * @param timestamp of image to build measurements for
   * @param image current image
   */
  bs_common::CameraMeasurementMsg
      BuildCameraMeasurement(const ros::Time& timestamp,
                             const sensor_msgs::Image& image);

  fuse_core::UUID device_id_; //!< The UUID of this device
  // loadable camera parameters
  bs_parameters::models::VisualFeatureTrackerParams params_;

  // subscribers
  ros::Subscriber image_subscriber_;

  // publishers
  ros::Publisher measurement_publisher_;

  // extrinsics object
  bs_common::ExtrinsicsLookupOnline& extrinsics_ =
      bs_common::ExtrinsicsLookupOnline::GetInstance();

  // callbacks for messages
  using ThrottledImageCallback =
      fuse_core::ThrottledMessageCallback<sensor_msgs::Image>;
  ThrottledImageCallback throttled_image_callback_;

  std::shared_ptr<beam_cv::Tracker> tracker_;
  std::shared_ptr<beam_cv::Descriptor> descriptor_;
  ros::Time prev_time_{0};
};

} // namespace bs_models
