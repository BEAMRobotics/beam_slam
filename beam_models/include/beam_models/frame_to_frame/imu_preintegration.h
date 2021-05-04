#pragma once

#include <sensor_msgs/Imu.h>

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
    Eigen::Matrix3d cov_w{Eigen::Matrix3d::Identity() * 1.5e-03};
    Eigen::Matrix3d cov_a{Eigen::Matrix3d::Identity() * 4.0e-03};
    Eigen::Matrix3d cov_bg{Eigen::Matrix3d::Identity() * 3.5e-05};
    Eigen::Matrix3d cov_ba{Eigen::Matrix3d::Identity() * 6.5e-05};
    std::string source{"IMUPREINTEGRATION"};
  };

  struct ImuData {
    ros::Time time;
    fuse_core::Vector3d angular_velocity;
    fuse_core::Vector3d linear_acceleration;

    ImuData() {
      linear_acceleration.setZero();
      angular_velocity.setZero();
    }
  };

  struct Delta {
    double t;
    Eigen::Quaterniond q;
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Matrix<double, 15, 15> cov; // ordered in q, p, v, bg, ba
  };

  struct Jacobian {
    Eigen::Matrix3d dq_dbg;
    Eigen::Matrix3d dp_dbg;
    Eigen::Matrix3d dp_dba;
    Eigen::Matrix3d dv_dbg;
    Eigen::Matrix3d dv_dba;
  };

  ImuPreintegration(const Params& params);

  ImuPreintegration(const Params& params, const Eigen::Vector3d& init_ba,
                    const Eigen::Vector3d& init_bg);

  ~ImuPreintegration() = default;

  void ClearBuffer() { imu_data_buffer_.clear(); }

  void PopulateBuffer(const sensor_msgs::Imu::ConstPtr& msg);

  void PopulateBuffer(const ImuData& imu_data);

  void ResetMotionEstimate();

  void SetStart(
      const ros::Time& t_start = ros::Time(0),  
      const Eigen::Quaterniond& orientation = Eigen::Quaterniond::Identity(),
      const Eigen::Vector3d& velocity = Eigen::Vector3d::Zero(),
      const Eigen::Vector3d& position = Eigen::Vector3d::Zero());

  ImuState GetImuState() const { return imu_state_i_; }    

  void Increment(double dt, const ImuData& data, bool compute_jacobian,
                 bool compute_covariance);

  bool Integrate(double t, bool compute_jacobian, bool compute_covariance);

  ImuState PredictState(ImuState& imu_state);

  beam_constraints::frame_to_frame::ImuState3DStampedTransaction
  RegisterNewImuPreintegrationFactor();

private:
  enum ErrorStateLocation {
    ES_Q = 0,
    ES_P = 3,
    ES_V = 6,
    ES_BG = 9,
    ES_BA = 12,
    ES_SIZE = 15
  };

  Params params_;
  double imu_state_prior_noise_{1e-9};
  fuse_core::Vector3d gravitational_acceleration_;

  Delta delta;
  Jacobian jacobian;
  std::vector<ImuData> imu_data_buffer_;

  ImuState imu_state_i_;
  Eigen::Vector3d ba_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d bg_{Eigen::Vector3d::Zero()};
};

}}  // namespace beam_models::frame_to_frame
