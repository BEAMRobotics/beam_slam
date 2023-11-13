#include <ros/ros.h>

#include <sensor_msgs/Image.h>

#include <beam_cv/OpenCVConversions.h>
#include <beam_cv/Utils.h>
#include <beam_cv/descriptors/ORBDescriptor.h>
#include <beam_cv/detectors/FASTSSCDetector.h>
#include <beam_cv/trackers/Trackers.h>

#include <bs_common/bs_msgs.h>
#include <bs_common/utils.h>

struct FeatureTracker {
  FeatureTracker() : nh_("~") {
    std::string descriptor_config;
    std::string detector_config;
    std::string tracker_config;
    std::string image_topic;

    if (!nh_.getParam("descriptor_config", descriptor_config)) {
      ROS_ERROR("Failed to load parameter 'descriptor_config'");
      throw std::runtime_error("Failed to load parameter 'descriptor_config'");
    }
    if (!nh_.getParam("detector_config", detector_config)) {
      ROS_ERROR("Failed to load parameter 'detector_config'");
      throw std::runtime_error("Failed to load parameter 'detector_config'");
    }
    if (!nh_.getParam("tracker_config", tracker_config)) {
      ROS_ERROR("Failed to load parameter 'tracker_config'");
      throw std::runtime_error("Failed to load parameter 'tracker_config'");
    }
    if (!nh_.getParam("image_topic", image_topic)) {
      ROS_ERROR("Failed to load parameter 'image_topic'");
      throw std::runtime_error("Failed to load parameter 'image_topic'");
    }

    // Initialize descriptor
    beam_cv::ORBDescriptor::Params descriptor_params;
    descriptor_params.LoadFromJson(descriptor_config);
    descriptor_ = std::make_shared<beam_cv::ORBDescriptor>(descriptor_params);

    // Initialize detector
    beam_cv::FASTSSCDetector::Params detector_params;
    detector_params.LoadFromJson(detector_config);
    std::shared_ptr<beam_cv::Detector> detector =
        std::make_shared<beam_cv::FASTSSCDetector>(detector_params);

    // Initialize tracker
    beam_cv::KLTracker::Params tracker_params;
    tracker_params.LoadFromJson(tracker_config);
    tracker_ = std::make_shared<beam_cv::KLTracker>(tracker_params, detector,
                                                    descriptor_, 3);

    // initialize pub and sub
    image_subscriber_ =
        nh_.subscribe(image_topic, 10, &FeatureTracker::ImageCallback, this);
    measurement_publisher_ = nh_.advertise<bs_common::CameraMeasurementMsg>(
        "/feature_tracker/visual_measurements", 10);
  }

  void ImageCallback(const sensor_msgs::Image::ConstPtr& msg) {
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

  bs_common::CameraMeasurementMsg
      BuildCameraMeasurement(const ros::Time& timestamp,
                             const sensor_msgs::Image& image) {
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
    camera_measurement.header.frame_id = "CAMERA";

    camera_measurement.descriptor_type = descriptor_->GetTypeString();
    camera_measurement.sensor_id = 0;
    camera_measurement.image = image;
    camera_measurement.landmarks = landmarks;
    // return message
    return camera_measurement;
  }

  ros::NodeHandle nh_;
  ros::Subscriber image_subscriber_;
  ros::Publisher measurement_publisher_;
  std::shared_ptr<beam_cv::Tracker> tracker_;
  std::shared_ptr<beam_cv::Descriptor> descriptor_;
  ros::Time prev_time_{0};
};

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "feature_tracker");
  FeatureTracker ft;
  ros::spin();

  return 0;
}
