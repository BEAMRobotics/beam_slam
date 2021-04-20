#pragma once

#include <sensor_msgs/Imu.h>

#include <beam_constraints/frame_to_frame/relative_pose_3d_stamped_transaction.h>
#include <beam_variables/imu_bias_stamped.h>

namespace beam_models { namespace frame_to_frame {

template <typename ConstraintType, typename PriorType>
using TransactionBase =
    beam_constraints::frame_to_frame::FrameToFrameTransactionBase<
        ConstraintType, PriorType>;

class ImuPreintegration {
public:
  struct Params {
    int buffer_size;
    double gravitational_acceleration;
    double initial_imu_acceleration_bias;
    double initial_imu_gyroscope_bias;
  };

  ImuPreintegration(const Params& params);

  ~ImuPreintegration() = default;

	void clearBuffers();

	void reserveBuffers();

	void populateBuffers(const sensor_msgs::Imu::ConstPtr& msg);

	void setFirstFrame(const sensor_msgs::Imu::ConstPtr& msg);

  void SetFixedCovariance(const fuse_core::Matrix6d& covariance);

	inline int getBufferSize() { return msg_time_buffer_.size(); }

private:
	Params params_;
	bool use_fixed_imu_noise_covariance_{false};

  std::vector<ros::Time> msg_time_buffer_;
  std::vector<fuse_core::Vector3d> angular_velocity_buffer_;
  std::vector<fuse_core::Vector3d> linear_acceleration_buffer_;
  std::vector<fuse_core::Matrix6d> imu_noise_buffer_;

  fuse_core::Matrix6d imu_noise_covariance_;
  fuse_core::Vector3d gravitational_acceleration_;

  fuse_variables::Orientation3DStamped::SharedPtr R_i_;
  fuse_variables::VelocityLinear3DStamped::SharedPtr V_i_;
  fuse_variables::Position3DStamped::SharedPtr P_i_;
  beam_variables::ImuBiasStamped::SharedPtr Ba_i_;
  beam_variables::ImuBiasStamped::SharedPtr Bg_i_;
};

}} // namespace beam_models::frame_to_frame
