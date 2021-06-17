#pragma once
#include <beam_utils/math.h>

namespace beam_models { namespace frame_to_frame {

struct IMUData {
  double t;        // timestamp
  Eigen::Vector3d w; // gyro measurement
  Eigen::Vector3d a; // accelerometer measurement
  /**
   * @brief Defualt Constructor
   */
  ImuData() = default;

  /**
   * @brief Constructor
   * @param msg sensor data
   */
  ImuData(const sensor_msgs::Imu::ConstPtr& msg) {
    t = msg->header.stamp.toSec();
    w[0] = msg->angular_velocity.x;
    w[1] = msg->angular_velocity.y;
    w[2] = msg->angular_velocity.z;
    a[0] = msg->linear_acceleration.x;
    a[1] = msg->linear_acceleration.y;
    a[2] = msg->linear_acceleration.z;
  }
};

struct PreIntegrator {
  struct Delta {
    double t;
    Eigen::Quaterniond q;
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Matrix<double, 15, 15> cov; // ordered in q, p, v, bg, ba
    Eigen::Matrix<double, 15, 15> sqrt_inv_cov;
  };

  struct Jacobian {
    Eigen::Matrix3d dq_dbg;
    Eigen::Matrix3d dp_dbg;
    Eigen::Matrix3d dp_dba;
    Eigen::Matrix3d dv_dbg;
    Eigen::Matrix3d dv_dba;
  };

  void Reset();
  void Increment(double t, const IMUData& data, const Eigen::Vector3d& bg,
                 const Eigen::Vector3d& ba, bool compute_jacobian,
                 bool compute_covariance);
  bool Integrate(double t, const Eigen::Vector3d& bg, const Eigen::Vector3d& ba,
                 bool compute_jacobian, bool compute_covariance);
  void ComputeSqrtInverseCovariance();

  Eigen::Matrix3d cov_w; // continuous noise covariance
  Eigen::Matrix3d cov_a;
  Eigen::Matrix3d cov_bg; // continuous random walk noise covariance
  Eigen::Matrix3d cov_ba;

  Delta delta;
  Jacobian jacobian;

  std::vector<IMUData> data;
};

}} // namespace beam_models::frame_to_frame
