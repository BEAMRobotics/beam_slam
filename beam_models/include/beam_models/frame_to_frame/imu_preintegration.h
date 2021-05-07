#pragma once

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
    ros::Time t_ros; // ros time

    ImuData() {
      t = 0; // time (s)
      w.setZero(); // gyro (rad/s)
      a.setZero(); // accel (m/s^2)
    }
  };

  ImuPreintegration(const Params& params);

  ImuPreintegration(const Params& params, const Eigen::Vector3d& init_bg,
                    const Eigen::Vector3d& init_ba);

  ~ImuPreintegration() = default;

  void ClearBuffer() { imu_data_buffer_.clear(); }

  void PopulateBuffer(const sensor_msgs::Imu::ConstPtr& msg);

  void PopulateBuffer(const ImuData& imu_data);

  void SetStart(const ros::Time& t_start,
                fuse_variables::Orientation3DStamped::SharedPtr orientation,
                fuse_variables::Position3DStamped::SharedPtr position,
                fuse_variables::VelocityLinear3DStamped::SharedPtr velocity);

  ImuState GetImuState() const { return imu_state_i_; }

  ImuState PredictState(const PreIntegrator& pre_integrator, ImuState& imu_state);

  beam_constraints::frame_to_frame::ImuState3DStampedTransaction
  GetIMUPreintegrationTransaction(
      const ros::Time& t_now,
      fuse_variables::Orientation3DStamped::SharedPtr orientation,
      fuse_variables::Position3DStamped::SharedPtr position);

private:
  Params params_;
  Eigen::Vector3d g_; 
  Eigen::Vector3d TEST{Eigen::Vector3d::Zero()};

  ImuState imu_state_i_;
  std::vector<ImuData> imu_data_buffer_;
  Eigen::Vector3d bg_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d ba_{Eigen::Vector3d::Zero()};
};

}}  // namespace beam_models::frame_to_frame
