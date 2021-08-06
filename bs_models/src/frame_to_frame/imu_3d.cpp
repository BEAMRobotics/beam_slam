#include <bs_models/frame_to_frame/imu_3d.h>
#include <bs_common/utils.h>

#include <fuse_core/transaction.h>
#include <pluginlib/class_list_macros.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(bs_models::frame_to_frame::Imu3D, fuse_core::SensorModel)

namespace bs_models {
namespace frame_to_frame {

Imu3D::Imu3D()
    : fuse_core::AsyncSensorModel(1),
      device_id_(fuse_core::uuid::NIL),
      throttled_callback_(
          std::bind(&Imu3D::process, this, std::placeholders::_1)) {}

void Imu3D::onInit() {
  params_.loadFromROS(private_node_handle_);

  // init frame initializer
  if (params_.frame_initializer_type == "ODOMETRY") {
    frame_initializer_ =
        std::make_unique<frame_initializers::OdometryFrameInitializer>(
            params_.frame_initializer_info, 100, 30,
            params_.sensor_frame_id_override);
  } else if (params_.frame_initializer_type == "POSEFILE") {
    frame_initializer_ =
        std::make_unique<frame_initializers::PoseFileFrameInitializer>(
            params_.frame_initializer_info);
  } else {
    const std::string error =
        "frame_initializer_type invalid. Options: ODOMETRY, POSEFILE";
    ROS_FATAL_STREAM(error);
    throw std::runtime_error(error);
  }

  // init imu preintegration
  ImuPreintegration::Params imu_preintegration_params{
    prior_noise : params_.prior_noise,
    gravity : Eigen::Vector3d(0, 0, -GRAVITY),
    cov_gyro_noise : params_.cov_gyro_noise,
    cov_accel_noise : params_.cov_accel_noise,
    cov_gyro_bias : params_.cov_gyro_bias,
    cov_accel_bias : params_.cov_accel_bias,
    source : name()
  };
  imu_preintegration_ =
      std::make_unique<ImuPreintegration>(imu_preintegration_params);

  // init elapsed time for key-frame selection
  t_elapsed_ = ros::Duration(0.0);
}

void Imu3D::onStart() {
  subscriber_ = node_handle_.subscribe<sensor_msgs::Imu>(
      ros::names::resolve(params_.input_topic), params_.queue_size,
      &ImuThrottledCallback::callback, &throttled_callback_,
      ros::TransportHints().tcpNoDelay(false));
};

void Imu3D::onStop() { subscriber_.shutdown(); }

void Imu3D::process(const sensor_msgs::Imu::ConstPtr& msg) {
  // add msg to buffer
  imu_preintegration_->AddToBuffer(*msg);
  ros::Time t_now = msg->header.stamp;

  // set start
  if (set_start_) {
    fuse_variables::Orientation3DStamped::SharedPtr R_WORLD_IMU;
    fuse_variables::Position3DStamped::SharedPtr t_WORLD_IMU;
    GetPose(R_WORLD_IMU, t_WORLD_IMU, t_now);
    imu_preintegration_->SetStart(t_now, R_WORLD_IMU, t_WORLD_IMU);
    set_start_ = false;
    t_prev_ = t_now;
  }

  // increment time
  t_elapsed_ += t_now - t_prev_;
  t_prev_ = t_now;

  // if key-frame ready
  if (t_elapsed_ >= ros::Duration(params_.key_frame_rate)) {
    t_elapsed_ = ros::Duration(0.0);
    ROS_INFO_STREAM("KEYFRAME AT: " << t_now.toSec());
  }
};

bool Imu3D::GetPose(
    fuse_variables::Orientation3DStamped::SharedPtr& R_WORLD_IMU,
    fuse_variables::Position3DStamped::SharedPtr& t_WORLD_IMU,
    const ros::Time& time) {
  Eigen ::Matrix4d T_WORLD_IMU;
  if (!frame_initializer_->GetEstimatedPose(T_WORLD_IMU, time,
                                            extrinsics_.GetImuFrameId())) {
    ROS_WARN(
        "Cannot get transform from imu to world for stamp: %.8f. Skipping key "
        "frame.",
        time.toSec());
    return false;
  }

  bs_common::EigenTransformToFusePose(T_WORLD_IMU, t_WORLD_IMU, R_WORLD_IMU);
  return true;
}

}  // namespace frame_to_frame
}  // namespace bs_models
