#include <bs_models/visual_feature_tracker.h>
#include <pluginlib/class_list_macros.h>

#include <beam_cv/OpenCVConversions.h>
#include <beam_cv/descriptors/ORBDescriptor.h>
#include <beam_cv/detectors/FASTSSCDetector.h>
#include <beam_cv/Utils.h>
#include <bs_common/utils.h>

#include <algorithm>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(bs_models::VisualFeatureTracker, fuse_core::SensorModel);

namespace bs_models {

VisualFeatureTracker::VisualFeatureTracker()
    : fuse_core::AsyncSensorModel(1),
      device_id_(fuse_core::uuid::NIL),
      throttled_image_callback_(std::bind(&VisualFeatureTracker::processImage,
                                          this, std::placeholders::_1)) {}

void VisualFeatureTracker::onInit() {
  // Read settings from the parameter sever
  device_id_ = fuse_variables::loadDeviceId(private_node_handle_);
  params_.loadFromROS(private_node_handle_);

  // Initialize descriptor
  beam_cv::ORBDescriptor::Params descriptor_params;
  descriptor_params.LoadFromJson(params_.descriptor_config);
  descriptor_ = std::make_shared<beam_cv::ORBDescriptor>(descriptor_params);

  // Initialize detector
  beam_cv::FASTSSCDetector::Params detector_params;
  detector_params.LoadFromJson(params_.detector_config);
  std::shared_ptr<beam_cv::Detector> detector =
      std::make_shared<beam_cv::FASTSSCDetector>(detector_params);

  // Initialize tracker
  beam_cv::KLTracker::Params tracker_params;
  tracker_params.LoadFromJson(params_.tracker_config);
  tracker_ = std::make_shared<beam_cv::KLTracker>(tracker_params, detector,
                                                  descriptor_, 3);
}

void VisualFeatureTracker::onStart() {
  // subscribe to image topic
  image_subscriber_ = private_node_handle_.subscribe<sensor_msgs::Image>(
      ros::names::resolve(params_.image_topic), 10,
      &ThrottledImageCallback::callback, &throttled_image_callback_,
      ros::TransportHints().tcpNoDelay(false));

  measurement_publisher_ =
      private_node_handle_.advertise<bs_common::CameraMeasurementMsg>(
          "visual_measurements", 5);
}

/************************************************************
 *                          Callbacks                       *
 ************************************************************/
void VisualFeatureTracker::processImage(
    const sensor_msgs::Image::ConstPtr& msg) {
  static ros::Time prev_track_publish(0.0);
  // track features in image
  cv::Mat image = beam_cv::OpenCVConversions::RosImgToMat(*msg);
  cv::Mat clahe_image = beam_cv::AdaptiveHistogram(image);
  tracker_->AddImage(clahe_image, msg->header.stamp);

  // delay publishing by one image to ensure that the tracks are actually
  // published
  if (prev_time_ == ros::Time(0)) {
    prev_time_ = msg->header.stamp;
    return;
  }

  const auto measurement_msg = BuildCameraMeasurement(prev_time_, *msg);
  measurement_publisher_.publish(measurement_msg);
  prev_time_ = msg->header.stamp;
}

bs_common::CameraMeasurementMsg VisualFeatureTracker::BuildCameraMeasurement(
    const ros::Time& timestamp, const sensor_msgs::Image& image) {
  static uint64_t measurement_id = 0;
  // build landmark measurements msg
  std::vector<bs_common::LandmarkMeasurementMsg> landmarks;
  std::vector<uint64_t> landmark_ids =
      tracker_->GetLandmarkIDsInImage(timestamp);
  for (auto& id : landmark_ids) {
    bs_common::LandmarkMeasurementMsg lm;
    lm.landmark_id = id;
    const cv::Mat descriptor = tracker_->GetDescriptor(timestamp, id);
    lm.descriptor.descriptor_type = descriptor_->GetTypeString();
    lm.descriptor.data = beam_cv::Descriptor::CvMatDescriptorToVector(
        descriptor, descriptor_->GetType());
    Eigen::Vector2d pixel = tracker_->Get(timestamp, id);
    lm.pixel_u = pixel[0];
    lm.pixel_v = pixel[1];
    landmarks.push_back(lm);
  }

  // build camera measurement msg
  bs_common::CameraMeasurementMsg camera_measurement;
  camera_measurement.header.seq = measurement_id++;
  camera_measurement.header.stamp = timestamp;
  camera_measurement.header.frame_id = extrinsics_.GetCameraFrameId();

  camera_measurement.descriptor_type = descriptor_->GetTypeString();
  camera_measurement.sensor_id = params_.sensor_id;
  camera_measurement.image = image;
  camera_measurement.landmarks = landmarks;
  // return message
  return camera_measurement;
}

} // namespace bs_models
