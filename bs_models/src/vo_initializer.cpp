#include <bs_models/vo_initializer.h>

#include <pluginlib/class_list_macros.h>

#include <beam_cv/OpenCVConversions.h>
#include <beam_cv/descriptors/Descriptors.h>
#include <beam_cv/detectors/Detectors.h>
#include <beam_cv/geometry/AbsolutePoseEstimator.h>
#include <beam_cv/geometry/RelativePoseEstimator.h>
#include <beam_cv/geometry/Triangulation.h>
#include <beam_utils/math.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(bs_models::VOInitializer, fuse_core::SensorModel)

namespace bs_models {

VOInitializer::VOInitializer() : fuse_core::AsyncSensorModel(1) {}

void VOInitializer::onInit() {
  // load parameters from ros
  calibration_params_.loadFromROS();
  vo_initializer_params_.loadFromROS(private_node_handle_);

  // advertise init path publisher
  results_publisher_ =
      private_node_handle_.advertise<bs_common::InitializedPathMsg>("result",
                                                                    10);
  // subscribe to imu topic
  image_subscriber_ =
      private_node_handle_.subscribe(vo_initializer_params_.image_topic, 100,
                                     &VOInitializer::processImage, this);

  // initialize pose refiner object with params
  pose_refiner_ = std::make_shared<beam_cv::PoseRefinement>(5e-2, true, 1.0);

  // Load camera model and Create Map object
  cam_model_ = beam_calibration::CameraModel::Create(
      calibration_params_.cam_intrinsics_path);

  // create visual map
  visual_map_ = std::make_shared<vision::VisualMap>(cam_model_);

  // Get descriptor type
  descriptor_type_ =
      beam_cv::DescriptorTypeStringMap[vo_initializer_params_.descriptor];
  for (auto& it : beam_cv::DescriptorTypeIntMap) {
    if (it.second == descriptor_type_) { descriptor_type_int_ = it.first; }
  }

  // Initialize descriptor
  std::shared_ptr<beam_cv::Descriptor> descriptor = beam_cv::Descriptor::Create(
      descriptor_type_, vo_initializer_params_.descriptor_config);

  // Initialize detector
  std::shared_ptr<beam_cv::Detector> detector = beam_cv::Detector::Create(
      beam_cv::DetectorTypeStringMap[vo_initializer_params_.detector],
      vo_initializer_params_.detector_config);

  // Initialize tracker
  beam_cv::KLTracker::Params tracker_params;
  tracker_params.LoadFromJson(vo_initializer_params_.tracker_config);
  tracker_ = std::make_shared<beam_cv::KLTracker>(
      tracker_params, detector, descriptor,
      vo_initializer_params_.tracker_window_size);
}

void VOInitializer::processImage(const sensor_msgs::Image::ConstPtr& msg) {
  if (initialization_complete_) return;

  ros::Time cur_time = msg->header.stamp;
  // push image time into queue
  times_.push_back(cur_time);
  if (times_.size() > vo_initializer_params_.tracker_window_size) {
    times_.pop_front();
  }

  // add image to tracker
  tracker_->AddImage(beam_cv::OpenCVConversions::RosImgToMat(*msg), cur_time);

  // try to initialize
  if (times_.size() >= 10) {
    // 1. Get matches between first and last image
    std::vector<Eigen::Vector2i, beam::AlignVec2i> p1_v;
    std::vector<Eigen::Vector2i, beam::AlignVec2i> p2_v;
    std::vector<uint64_t> ids = tracker_->GetLandmarkIDsInImage(cur_time);
    double total_parallax = 0.0;
    double num_correspondences = 0.0;
    for (auto& id : ids) {
      try {
        Eigen::Vector2i p1 = tracker_->Get(times_.front(), id).cast<int>();
        Eigen::Vector2i p2 = tracker_->Get(cur_time, id).cast<int>();
        p1_v.push_back(p1);
        p2_v.push_back(p2);
        double d = beam::distance(p1, p2);
        total_parallax += d;
        num_correspondences += 1.0;
      } catch (const std::out_of_range& oor) {}
    }
    double parallax = total_parallax / num_correspondences;

    // if parallax is good then try to initialize
    if (parallax > vo_initializer_params_.parallax) {
      beam::opt<Eigen::Matrix4d> T_c2_c1 =
          beam_cv::RelativePoseEstimator::RANSACEstimator(
              cam_model_, cam_model_, p1_v, p2_v,
              beam_cv::EstimatorMethod::SEVENPOINT);

      int num_inliers = beam_cv::CheckInliers(cam_model_, cam_model_, p1_v,
                                              p2_v, Eigen::Matrix4d::Identity(),
                                              T_c2_c1.value(), 5);

      if ((num_inliers / (double)p1_v.size()) > 0.8) {
        // 3. Add each pose to the visual map (taking first pose as the
        // transform from baselink to camera)
        // 4. Triangulate features between them and add to visual map
        // 5. Localize every other frame between these two

        initialization_complete_ = true;
      }
    }
  }
}

void VOInitializer::PublishResults() {
  bs_common::InitializedPathMsg msg;

  for (uint32_t i = 0; i < trajectory_.size(); i++) {
    Eigen::Matrix4d T = trajectory_[i];

    std_msgs::Header header;
    header.frame_id = extrinsics_.GetBaselinkFrameId();
    header.seq = i;
    header.stamp = times_[i];

    geometry_msgs::Point position;
    position.x = T(0, 3);
    position.y = T(1, 3);
    position.z = T(2, 3);

    Eigen::Matrix3d R = T.block(0, 0, 3, 3);
    Eigen::Quaterniond q(R);

    geometry_msgs::Quaternion orientation;
    orientation.x = q.x();
    orientation.y = q.y();
    orientation.z = q.z();
    orientation.w = q.w();

    geometry_msgs::PoseStamped pose;
    pose.header = header;
    pose.pose.position = position;
    pose.pose.orientation = orientation;
    msg.poses.push_back(pose);
  }
  results_publisher_.publish(msg);
}

} // namespace bs_models
