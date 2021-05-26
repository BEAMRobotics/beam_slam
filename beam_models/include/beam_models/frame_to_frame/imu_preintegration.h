#pragma once

#include <queue>

#include <sensor_msgs/Imu.h>
#include <slamtools/preintegrator.h>

#include <beam_constraints/frame_to_frame/imu_state_3d_stamped_transaction.h>
#include <beam_models/frame_to_frame/imu_state.h>

namespace beam_models { namespace frame_to_frame {

template <typename ConstraintType, typename PriorType>
using TransactionBase =
    beam_constraints::frame_to_frame::FrameToFrameTransactionBase<
        ConstraintType, PriorType>;

class ImuPreintegration {
public:
  struct Params {
    double gravitational_acceleration{9.80665};
    double prior_noise{1e-9};
    Eigen::Matrix3d cov_gyro_noise{Eigen::Matrix3d::Identity() * 1.5e-03};
    Eigen::Matrix3d cov_accel_noise{Eigen::Matrix3d::Identity() * 4.0e-03};
    Eigen::Matrix3d cov_gyro_bias{Eigen::Matrix3d::Identity() * 3.5e-05};
    Eigen::Matrix3d cov_accel_bias{Eigen::Matrix3d::Identity() * 6.5e-05};
    std::string source{"IMUPREINTEGRATION"};
  };

  // wrap IMUData from slamtools
  struct ImuData : public IMUData {
    ros::Time t_ros;

    ImuData() = default;

    ImuData(const sensor_msgs::Imu::ConstPtr& msg) {
      t_ros = msg->header.stamp;
      t = msg->header.stamp.toSec();
      w[0] = msg->angular_velocity.x;
      w[1] = msg->angular_velocity.y;
      w[2] = msg->angular_velocity.z;
      a[0] = msg->linear_acceleration.x;
      a[1] = msg->linear_acceleration.y;
      a[2] = msg->linear_acceleration.z;
    }
  };

  ImuPreintegration(const Params& params);

  ImuPreintegration(const Params& params, const Eigen::Vector3d& init_bg,
                    const Eigen::Vector3d& init_ba);

  ~ImuPreintegration() = default;

  void ClearBuffer();

  void PopulateBuffer(const sensor_msgs::Imu::ConstPtr& msg);

  void PopulateBuffer(const ImuData& imu_data);

  void SetStart(
      const ros::Time& t_start,
      fuse_variables::Orientation3DStamped::SharedPtr orientation = nullptr,
      fuse_variables::Position3DStamped::SharedPtr position = nullptr,
      fuse_variables::VelocityLinear3DStamped::SharedPtr velocity = nullptr);

  ImuState PredictState(const PreIntegrator& pre_integrator,
                        const ImuState& imu_state_curr);

  Eigen::Matrix<double, 16, 1> CalculateRelativeChange(
    const ImuState& imu_state_curr);    

  ImuState GetImuState() const { return imu_state_i_; }                    

  Eigen::Matrix4d GetPose(const ros::Time& t_now);

  beam_constraints::frame_to_frame::ImuState3DStampedTransaction
  RegisterNewImuPreintegratedFactor(
      const ros::Time& t_now,
      fuse_variables::Orientation3DStamped::SharedPtr orientation = nullptr,
      fuse_variables::Position3DStamped::SharedPtr position = nullptr);

private:
  void SetPreintegrator();

  void ResetPreintegrator();

  Params params_;
  Eigen::Vector3d g_;
  bool first_window_{true};

  ImuState imu_state_i_;                 // first key frame
  ImuState imu_state_k_;                 // intermediate frame
  PreIntegrator pre_integrator_ij;       // Preintegrate between key frames
  std::queue<ImuData> imu_data_buffer_;  // store imu data
  Eigen::Vector3d bg_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d ba_{Eigen::Vector3d::Zero()};
};

}}  // namespace beam_models::frame_to_frame