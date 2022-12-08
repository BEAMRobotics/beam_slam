#include <bs_models/slam_initialization.h>

#include <bs_models/vision/utils.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(bs_models::SLAMInitialization, fuse_core::SensorModel)

namespace bs_models {

using namespace vision;

SLAMInitialization::SLAMInitialization()
    : fuse_core::AsyncSensorModel(1),
      device_id_(fuse_core::uuid::NIL),
      throttled_image_callback_(std::bind(&SLAMInitialization::processImage,
                                          this, std::placeholders::_1)) {}

void SLAMInitialization::onInit() {
  // Read settings from the parameter sever
  device_id_ = fuse_variables::loadDeviceId(private_node_handle_);
  calibration_params_.loadFromROS();
  slam_initialization_params_.loadFromROS(private_node_handle_);

  // Load camera model and Create Map object
  cam_model_ = beam_calibration::CameraModel::Create(
      calibration_params_.cam_intrinsics_path);
  cam_model_->InitUndistortMap();

  // create optimization graph
  local_graph_ = std::make_shared<fuse_graphs::HashGraph>();

  // create visual map
  visual_map_ = std::make_shared<vision::VisualMap>(cam_model_);
}

void SLAMInitialization::onStart() {
  // subscribe to topics
  visual_measurement_subscriber_ =
      private_node_handle_.subscribe<sensor_msgs::Image>(
          ros::names::resolve(
              slam_initialization_params_.visual_measurement_topic),
          1000, &ThrottledMeasuremenCallback::callback,
          &throttled_measurement_callback_,
          ros::TransportHints().tcpNoDelay(false));

  imu_subscriber_ = private_node_handle_.subscribe<sensor_msgs::Image>(
      ros::names::resolve(slam_initialization_params_.imu_topic), 1000,
      &ThrottledIMUCallback::callback, &throttled_imu_callback_,
      ros::TransportHints().tcpNoDelay(false));

  lidar_subscriber_ = private_node_handle_.subscribe<sensor_msgs::Image>(
      ros::names::resolve(slam_initialization_params_.lidar_topic), 1000,
      &ThrottledLidarCallback::callback, &throttled_lidar_callback_,
      ros::TransportHints().tcpNoDelay(false));
}

void SLAMInitialization::processMeasurements(
    const CameraMeasurementMsg::ConstPtr& msg) {
  // put measurements into landmark container
  for (const auto& lm : msg->landmarks) {
    Eigen::Vector2d landmark(static_cast<double>(lm.pixel_u),
                             static_cast<double>(lm.pixel_v));

    cv::Mat landmark_descriptor = beam_cv::Descriptor::CreateDescriptor(
        lm.descriptor, msg->descriptor_type);

    landmark_container_.Emplace(msg->header.time, msg->sensor_id,
                                lm.landmark_id, img_times_.size(), landmark,
                                landmark_descriptor);
  }
  img_times_.push_back(msg->header.time);

  if (mode_ != InitMode::VISUAL) { return; }

  if (img_times_.size() < 2) { return; }

  const auto avg_parallax =
      computeParallax(img_times_.front(), img_times_.back());

  if (avg_parallax < slam_initialization_params_.min_parallax) { return; }

  if (!extrinsics_.GetT_CAMERA_BASELINK(T_cam_baselink_)) {
    ROS_ERROR("Unable to get camera to baselink transform.");
    return;
  }

  init_path_ = bs_models::vision::computePathWithVision(landmark_container_,
                                                        T_cam_baselink_);

  if (!init_path_.empty())

  // initialize();
}

void SLAMInitialization::processIMU(const sensor_msgs::Imu::ConstPtr& msg) {
  imu_buffer_.push(*msg);
  if (mode_ != InitMode::FRAMEINIT) { return; }

  if (!frame_initializer_) { return; }

  Eigen::Matrix4d T_WORLD_BASELINK;
  bool success = frame_initializer_->GetEstimatedPose(
      T_WORLD_BASELINK, msg->header.time, extrinsics_.GetBaselinkFrameId());
  init_path_[msg->header.time.toNSec()] = T_WORLD_BASELINK;

  const auto [first_time, first_pose] = init_path_.begin();
  const auto [current_time, current_pose] = init_path_.rbegin();
  if (beam::PassedMotionThreshold(
          first_pose, current_pose, 0.0,
          slam_initialization_params_.min_trajectory_length, true, true,
          false)) {
    // initialize();
  }
}

void SLAMInitialization::processLidar(
    const sensor_msgs::PointCloud2::ConstPtr& msg) {
  lidar_buffer_.push(*msg);
  if (mode_ != InitMode::LIDAR) { return; }

  // compute path
  // check if its long enough
  // initialize
}

double SLAMInitialization::computeParallax(const ros::Time& t1,
                                           const ros::Time& t2) {
  // check if parallax is large enough
  std::vector<uint64_t> frame1_ids =
      landmark_container_.GetLandmarkIDsInImage(t1);
  double total_parallax = 0.0;
  double num_correspondences = 0.0;
  for (const auto& id : frame1_ids) {
    try {
      Eigen::Vector2d p1 = landmark_container_.Get(t1, id);
      Eigen::Vector2d p2 = landmark_container_.Get(t2, id);
      double d = beam::distance(p1, p2);
      total_parallax += d;
      num_correspondences += 1.0;
    } catch (const std::out_of_range& oor) {}
  }

  return total_parallax / num_correspondences;
}

} // namespace bs_models
