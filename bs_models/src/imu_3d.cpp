#include <bs_models/imu_3d.h>

#include <beam_utils/filesystem.h>
#include <fuse_core/transaction.h>
#include <pluginlib/class_list_macros.h>

#include <bs_common/utils.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(bs_models::Imu3D, fuse_core::SensorModel)

namespace bs_models {

Imu3D::Imu3D()
    : fuse_core::AsyncSensorModel(1), device_id_(fuse_core::uuid::NIL),
      throttled_callback_(
          std::bind(&Imu3D::process, this, std::placeholders::_1)) {}

void Imu3D::onInit() {
  params_.loadFromROS(private_node_handle_);
  calibration_params_.loadFromROS();

  // init frame initializer
  if (params_.frame_initializer_type == "ODOMETRY") {
    frame_initializer_ =
        std::make_unique<frame_initializers::OdometryFrameInitializer>(
            params_.frame_initializer_info, 100, 2 * params_.lag_duration,
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

  // init velocities and biases
  fuse_variables::VelocityLinear3DStamped init_vel;
  init_vel.x() = params_.init_velocity_x;
  init_vel.y() = params_.init_velocity_y;
  init_vel.z() = params_.init_velocity_z;

  init_velocity_ =
      fuse_variables::VelocityLinear3DStamped::make_shared(init_vel);
  double bg = params_.init_gyro_bias;
  double ba = params_.init_accel_bias;
  init_gyro_bias_ << bg, bg, bg;
  init_accel_bias_ << ba, ba, ba;

  // init imu preintegration
  ImuPreintegration::Params imu_preintegration_params;
  imu_preintegration_params.LoadFromJSON(
      calibration_params_.imu_intrinsics_path);
  imu_preintegration_params.cov_prior_noise = params_.cov_prior_noise;

  imu_preintegration_ = std::make_unique<ImuPreintegration>(
      imu_preintegration_params, init_gyro_bias_, init_accel_bias_);

  // init elapsed time for key-frame selection
  t_elapsed_ = ros::Duration(0.0);

  // init lag duration for frame initialization
  t_lag_ = ros::Duration(params_.lag_duration);
}

void Imu3D::onStart() {
  subscriber_ = private_node_handle_.subscribe<sensor_msgs::Imu>(
      ros::names::resolve(params_.input_topic), params_.queue_size,
      &ThrottledCallback::callback, &throttled_callback_,
      ros::TransportHints().tcpNoDelay(false));
};

void Imu3D::onStop() { subscriber_.shutdown(); }

void Imu3D::process(const sensor_msgs::Imu::ConstPtr &msg) {
  // add msg to preintegration buffers
  imu_preintegration_->AddToBuffer(*msg);
  ros::Time t_proc = msg->header.stamp;

  // add time to queue for proper frame initialization
  t_buffer_.push(t_proc);

  // if lag duration exceeded
  if (ros::Duration(t_buffer_.back() - t_buffer_.front()) >= t_lag_) {
    // process first out
    ros::Time t_now = t_buffer_.front();
    t_buffer_.pop();

    // set first key frame
    if (set_start_) {
      // get pose from frame initializer
      fuse_variables::Orientation3DStamped::SharedPtr R_WORLD_IMU;
      fuse_variables::Position3DStamped::SharedPtr t_WORLD_IMU;
      GetEstimatedPose(R_WORLD_IMU, t_WORLD_IMU, t_now);

      // set start of preintegration from estimated pose
      imu_preintegration_->SetStart(t_now, R_WORLD_IMU, t_WORLD_IMU,
                                    init_velocity_);

      set_start_ = false;
      t_prev_ = t_now;
    }

    // increment time
    t_elapsed_ += t_now - t_prev_;
    t_prev_ = t_now;

    // if key-frame ready
    if (t_elapsed_ >= ros::Duration(params_.key_frame_rate)) {
      // reset elapsed time
      t_elapsed_ = ros::Duration(0.0);

      // get pose from frame initializer
      fuse_variables::Orientation3DStamped::SharedPtr R_WORLD_IMU;
      fuse_variables::Position3DStamped::SharedPtr t_WORLD_IMU;
      GetEstimatedPose(R_WORLD_IMU, t_WORLD_IMU, t_now);

      // send transaction
      fuse_core::Transaction::SharedPtr new_transaction =
          imu_preintegration_->RegisterNewImuPreintegratedFactor(
              t_now, R_WORLD_IMU, t_WORLD_IMU);

      if (new_transaction != nullptr) {
        ROS_INFO("Sending transaction.");
        try {
          sendTransaction(new_transaction);
        } catch (const std::exception &e) {
          ROS_WARN("Cannot send transaction. Error: %s", e.what());
        }
      }
    }
  }
};

bool Imu3D::GetEstimatedPose(
    fuse_variables::Orientation3DStamped::SharedPtr &R_WORLD_IMU,
    fuse_variables::Position3DStamped::SharedPtr &t_WORLD_IMU,
    const ros::Time &time) {
  Eigen::Matrix4d T_WORLD_IMU;
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

} // namespace bs_models
