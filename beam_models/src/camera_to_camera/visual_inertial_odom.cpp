#include <beam_models/camera_to_camera/visual_inertial_odom.h>
// fuse
#include <fuse_core/transaction.h>
#include <pluginlib/class_list_macros.h>
// other
#include <cv_bridge/cv_bridge.h>
// libbeam
#include <beam_cv/descriptors/Descriptors.h>
#include <beam_cv/detectors/Detectors.h>
#include <beam_cv/geometry/Triangulation.h>
#include <beam_cv/matchers/Matchers.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(beam_models::camera_to_camera::VisualInertialOdom,
                       fuse_core::SensorModel)

namespace beam_models { namespace camera_to_camera {

VisualInertialOdom::VisualInertialOdom()
    : fuse_core::AsyncSensorModel(1), device_id_(fuse_core::uuid::NIL) {}

void VisualInertialOdom::onInit() {
  // Read settings from the parameter sever
  device_id_ = fuse_variables::loadDeviceId(private_node_handle_);
  params_.loadFromROS(private_node_handle_);
  /***********************************************************
   *        Load camera model and Create Map object          *
   ***********************************************************/
  this->cam_model_ =
      beam_calibration::CameraModel::Create(params_.cam_intrinsics_path);
  this->visual_map_ =
      std::make_shared<VisualMap>(this->source_, this->cam_model_);
  /***********************************************************
   *              Initialize tracker variables               *
   ***********************************************************/
  std::shared_ptr<beam_cv::Matcher> matcher =
      std::make_shared<beam_cv::FLANNMatcher>(beam_cv::FLANN::KDTree, 0.8, true,
                                              true, cv::FM_RANSAC, 1);
  std::shared_ptr<beam_cv::Descriptor> descriptor =
      std::make_shared<beam_cv::ORBDescriptor>();
  std::shared_ptr<beam_cv::Detector> detector =
      std::make_shared<beam_cv::FASTDetector>(300);
  this->tracker_ = std::make_shared<beam_cv::Tracker>(
      detector, descriptor, matcher, params_.window_size);
  /***********************************************************
   *                  Subscribe to topics                    *
   ***********************************************************/
  image_subscriber_ = private_node_handle_.subscribe(
      params_.image_topic, 1000, &VisualInertialOdom::processImage, this);
  imu_subscriber_ = private_node_handle_.subscribe(
      params_.imu_topic, 10000, &VisualInertialOdom::processIMU, this);
  /***********************************************************
   *               Create initializer object                 *
   ***********************************************************/
  Eigen::Matrix4d T_body_cam;
  T_body_cam << 0.0148655429818, -0.999880929698, 0.00414029679422,
      -0.0216401454975, 0.999557249008, 0.0149672133247, 0.025715529948,
      -0.064676986768, -0.0257744366974, 0.00375618835797, 0.999660727178,
      0.00981073058949, 0.0, 0.0, 0.0, 1.0;
  Eigen::Matrix4d T_body_imu = Eigen::Matrix4d::Identity();
  Eigen::Vector4d imu_intrinsics(params_.imu_intrinsics.data());
  initializer_ = std::make_shared<VIOInitializer>(cam_model_, T_body_cam,
                                                  T_body_imu, imu_intrinsics);
}

void VisualInertialOdom::processImage(const sensor_msgs::Image::ConstPtr& msg) {
  // get message info
  image_buffer_.push(*msg);
  sensor_msgs::Image img_msg = image_buffer_.front();
  ros::Time img_time = img_msg.header.stamp;
  /**************************************************************************
   *              Add IMU messages to buffer or initializer                 *
   **************************************************************************/
  while (imu_buffer_.front().header.stamp < img_time && !imu_buffer_.empty()) {
    sensor_msgs::Imu imu_msg = imu_buffer_.front();
    ros::Time imu_time = imu_msg.header.stamp;
    Eigen::Vector3d ang_vel{imu_msg.angular_velocity.x,
                            imu_msg.angular_velocity.y,
                            imu_msg.angular_velocity.z};
    Eigen::Vector3d lin_accel{imu_msg.linear_acceleration.x,
                              imu_msg.linear_acceleration.y,
                              imu_msg.linear_acceleration.z};
    if (!initializer_->Initialized()) {
      temp_imu_buffer_.push(imu_msg);
      initializer_->AddIMU(ang_vel, lin_accel, imu_time);
    } else {
      // preintegrator.PopulateBuffer(msg);
    }
    imu_buffer_.pop();
  }
  /**************************************************************************
   *                    Add Image to map or initializer                     *
   **************************************************************************/
  if (!imu_buffer_.empty()) {
    img_num_++;
    cv::Mat image = this->extractImage(img_msg);
    tracker_->AddImage(image, img_time);
    if (!initializer_->Initialized()) {
      if (initializer_->AddImage(image, img_time)) {
        // this transaction adds the first pose, adds the initial map points and
        // initializes the preintegrator object
        auto init_transaction = this->initMap();
        sendTransaction(init_transaction);
        // send messages in temp imu buffer to preint object
        // register the current frame against the map
        auto frame_transaction = this->registerFrame(img_time);
        sendTransaction(init_transaction);
      } else {
        std::queue<sensor_msgs::Imu>().swap(temp_imu_buffer_);
      }
    } else {
      // register the current frame against the map
      auto transaction = this->registerFrame(img_time);
      sendTransaction(transaction);
    }
    image_buffer_.pop();
  }
}

void VisualInertialOdom::processIMU(const sensor_msgs::Imu::ConstPtr& msg) {
  // indiscriminantly push imu messages onto its buffer
  imu_buffer_.push(*msg);
}

void VisualInertialOdom::onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph) {
  this->visual_map_->updateGraph(graph);
}

void VisualInertialOdom::onStop() {}

std::shared_ptr<fuse_core::Transaction> VisualInertialOdom::initMap() {
  auto transaction = fuse_core::Transaction::make_shared();
  // get poses from initialization and add to transaction
  std::map<unsigned long, Eigen::Matrix4d> poses = initializer_->GetPoses();
  ros::Time cur_time;
  for (auto& pose : poses) {
    cur_time.fromNSec(pose.first);
    Eigen::Quaterniond q;
    Eigen::Vector3d p;
    beam::TransformMatrixToQuaternionAndTranslation(pose.second, q, p);
    this->visual_map_->addOrientation(q, cur_time, transaction);
    this->visual_map_->addPosition(p, cur_time, transaction);
  }
  int num_lm = 0;
  // find landmarks in the frame
  std::vector<uint64_t> lm_ids = tracker_->GetLandmarkIDsInImage(cur_time);
  for (int i = 0; i < lm_ids.size(); i++) {
    uint64_t id = lm_ids[i];
    // get landmark track
    beam_cv::FeatureTrack track = tracker_->GetTrack(id);
    // get pixel of this landmark in the current image
    Eigen::Vector2d pixel_measurement = tracker_->Get(cur_time, id);
    // triangulate the track and add to transaction
    beam::opt<Eigen::Vector3d> point = this->triangulate(track);
    if (point.has_value()) {
      num_lm++;
      this->visual_map_->addLandmark(point.value(), id, transaction);
      this->visual_map_->addConstraint(cur_time, id, pixel_measurement,
                                       transaction);
    }
  }
  ROS_INFO("%d Initialized Map Points.", num_lm);
  /***
   * Initialize imu preintegrator here
   */
  return transaction;
}

std::shared_ptr<fuse_core::Transaction>
    VisualInertialOdom::registerFrame(const ros::Time& img_time) {
  /*
  1. Get relative pose estimate from preintegrator
  2. Start keyframe decision:
    1. If time t has elapsed since last kf, then this is a kf
    2. else if (parallax > n && matches > m) && translational movement > x,
  then this is a kf
  3. if frame is a keyframe then:
    1. if it has > n matches with last keyframe,
      1. get pose estimate with pnp
      2. add constraints to the landmarks it sees that are in the map
      3. triangulate the landmarks it sees that are not in the map
    2. else, compute pose with the imu given relative pose and add just that to
  the map
    3. Signal scan matcher that a keyframe was added at time t
  */
}

cv::Mat VisualInertialOdom::extractImage(const sensor_msgs::Image& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  return cv_ptr->image;
}

beam::opt<Eigen::Vector3d>
    VisualInertialOdom::triangulate(beam_cv::FeatureTrack track) {
  // first check if its already been triangulated
  uint64_t id = track[0].landmark_id;
  fuse_variables::Position3D::SharedPtr lm = this->visual_map_->getLandmark(id);
  if (lm) {
    Eigen::Vector3d p(lm->data());
    return p;
  }
  // if it hasnt then manually triangulate
  if (track.size() >= 2) {
    std::vector<Eigen::Matrix4d> T_cam_world_v;
    std::vector<Eigen::Vector2i> pixels;
    for (auto& measurement : track) {
      fuse_variables::Position3DStamped::SharedPtr p =
          this->visual_map_->getPosition(measurement.time_point);
      fuse_variables::Orientation3DStamped::SharedPtr q =
          this->visual_map_->getOrientation(measurement.time_point);
      if (p && q) {
        Eigen::Vector3d position(p->data());
        Eigen::Quaterniond orientation(q->data());
        Eigen::Matrix4d T;
        beam::QuaternionAndTranslationToTransformMatrix(orientation, position,
                                                        T);
        pixels.push_back(measurement.value.cast<int>());
        T_cam_world_v.push_back(T);
      }
    }
    beam::opt<Eigen::Vector3d> point = beam_cv::Triangulation::TriangulatePoint(
        this->cam_model_, T_cam_world_v, pixels);
    return point;
  }
  return {};
}

}} // namespace beam_models::camera_to_camera
