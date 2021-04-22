#pragma once

#include <sensor_msgs/Imu.h>

#include <beam_constraints/frame_to_frame/imu_state_3d_stamped_transaction.h>
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
		std::string source{"IMUPREINTERATION"};
		const fuse_core::UUID& device_id = fuse_core::uuid::NIL;
  };

	struct ImuData {
		ros::Time time;
		fuse_core::Vector3d linear_acceleration;
		fuse_core::Vector3d angular_velocity;
		fuse_core::Matrix6d noise_covariance;

		ImuData() {
			linear_acceleration.setZero();
			angular_velocity.setZero();
			noise_covariance.setZero();
		}
	};

  ImuPreintegration(const Params& params);

  ~ImuPreintegration() = default;

	void clearBuffer() { imu_data_buffer_.clear(); }

	void reserveBuffer() { imu_data_buffer_.reserve(params_.buffer_size); }

	void populateBuffer(const sensor_msgs::Imu::ConstPtr& msg);

  void SetFixedCovariance(const fuse_core::Matrix6d& covariance);

	inline int getBufferSize() { return imu_data_buffer_.size(); }

	beam_constraints::frame_to_frame::ImuState3DStampedTransaction
      RegisterNewImuPreintegrationFactor();

private:
	Params params_;
	bool first_window_{true};
	bool use_fixed_imu_noise_covariance_{false};
	fuse_core::Matrix6d imu_noise_covariance_;
	fuse_core::Vector3d gravitational_acceleration_;

	std::vector<ImuData> imu_data_buffer_;

  fuse_variables::Orientation3DStamped::SharedPtr R_i_;
  fuse_variables::VelocityLinear3DStamped::SharedPtr V_i_;
  fuse_variables::Position3DStamped::SharedPtr P_i_;
  beam_variables::ImuBiasStamped::SharedPtr Ba_i_;
  beam_variables::ImuBiasStamped::SharedPtr Bg_i_;
};

}} // namespace beam_models::frame_to_frame
