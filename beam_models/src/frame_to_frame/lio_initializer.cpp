#include <beam_models/frame_to_frame/lio_initializer.h>

#include <pluginlib/class_list_macros.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(beam_models::frame_to_frame::LioInitializer,
                       fuse_core::SensorModel)

namespace beam_models {
namespace frame_to_frame {

LioInitializer::LioInitializer() : fuse_core::AsyncSensorModel(1) {}

void LioInitializer::onInit() {
  // Read settings from the parameter sever
  params_.loadFromROS(private_node_handle_);

  // subscribe to imu topic
  imu_subscriber_ = private_node_handle_.subscribe(
      params_.imu_topic, 1000, &LioInitializer::processIMU, this);

  // subscribe to lidar topic
  lidar_subscriber_ = private_node_handle_.subscribe(
      params_.lidar_topic, 10, &LioInitializer::processLidar, this);

  // init imu preintegration
  ImuPreintegration::Params imu_preint_params;
  if (params_.nominal_gravity_direction == "-Z") {
    imu_preint_params.gravity = Eigen::Vector3d(0, 0, -GRAVITY);
  } else if (params_.nominal_gravity_direction == "+Z") {
    imu_preint_params.gravity = Eigen::Vector3d(0, 0, GRAVITY);
  } else if (params_.nominal_gravity_direction == "-X") {
    imu_preint_params.gravity = Eigen::Vector3d(-GRAVITY, 0, 0);
  } else if (params_.nominal_gravity_direction == "+X") {
    imu_preint_params.gravity = Eigen::Vector3d(GRAVITY, 0, 0);
  } else if (params_.nominal_gravity_direction == "-Y") {
    imu_preint_params.gravity = Eigen::Vector3d(0, -GRAVITY, 0);
  } else if (params_.nominal_gravity_direction == "+Y") {
    imu_preint_params.gravity = Eigen::Vector3d(0, GRAVITY, 0);
  } else {
    ROS_ERROR(
        "Invalid nominal_gravity_direction params. Options: -Z, +Z, -Y, +Y, "
        "-X, +X. Using default: -Z");
    imu_preint_params.gravity = Eigen::Vector3d(0, 0, -GRAVITY);
  }
  imu_preintegration_ = std::make_unique<ImuPreintegration>(imu_preint_params);
}

void LioInitializer::processIMU(const sensor_msgs::Imu::ConstPtr& msg) {
  if (initialization_complete_) {
    return;
  }

  beam_common::IMUData new_data(msg);
  imu_preintegration_->AddToBuffer(new_data);
}

void LioInitializer::processLidar(
    const sensor_msgs::PointCloud2::ConstPtr& msg) {
  if (initialization_complete_) {
    return;
  }

  ROS_DEBUG("Received incoming scan");
  PointCloudPtr cloud_current = beam::ROSToPCL(*msg);

  // init first message
  if (keyframe_start_time_.toSecs() == 0) {
    keyframe_cloud_ += *cloud_current;
    keyframe_start_time_ = msg->header.stamp;
    Eigen::Matrix4d T_WORLD_IMUFRAME =
        imu_preintegration_->GetPose(msg->header.stamp);
    T_KEYFRAME_WORLD_ = beam::InvertTranform(T_WORLD_IMUFRAME);
    return;
  }

  if (msg->header.stamp - keyframe_start_time_ > params_.aggregation_time) {
    ProcessCurrentKeyframe();
    keyframe_cloud_ = *cloud_current;
    keyframe_start_time_ = msg->header.stamp;
    Eigen::Matrix4d T_WORLD_IMUFRAME =
        imu_preintegration_->GetPose(msg->header.stamp);
    T_KEYFRAME_WORLD_ = beam::InvertTranform(T_WORLD_IMUFRAME);
  }

  AddPointcloud(*cloud_current, msg->header.stamp);
}

void LioInitializer::onStop() {}

void LioInitializer::ProcessCurrentKeyframe() {
  //
}

void LioInitializer::AddPointcloud(const PointCloud& cloud,
                                   const ros::Time& time) {
  if (undistort_scans_) {
    Eigen::Matrix4d T_WORLD_IMUNOW = imu_preintegration_->GetPose(time);
    Eigen::Matrix4d T_KEYFRAME_CLOUDNOW =
        T_KEYFRAME_WORLD_ * T_WORLD_IMUNOW * extrinsics_.GetT_IMU_LIDAR();
    PointCloud cloud_converted;
    pcl::transformPointCloud(cloud, cloud_converted, T_KEYFRAME_CLOUDNOW);
    current_aggregate_ += cloud_converted;
  } else {
    current_aggregate_ += cloud;
  }
}

}  // namespace frame_to_frame
}  // namespace beam_models
