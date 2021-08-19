#pragma once

#include <beam_utils/math.h>
#include <beam_utils/time.h>
#include <sensor_msgs/Imu.h>

namespace bs_common {

/**
 * @brief Enum class representing order of states in covariance matrix
 */
enum ErrorStateLocation {
  ES_Q = 0,
  ES_P = 3,
  ES_V = 6,
  ES_BG = 9,
  ES_BA = 12,
  ES_SIZE = 15
};

/**
 * @brief Struct representing a single imu measurement
 */
struct IMUData {
  /**
   * @brief Default Constructor
   */
  IMUData() = default;

  /**
   * @brief Constructor
   * @param msg sensor data
   */
  IMUData(const sensor_msgs::Imu& msg) {
    t = msg.header.stamp;
    w[0] = msg.angular_velocity.x;
    w[1] = msg.angular_velocity.y;
    w[2] = msg.angular_velocity.z;
    a[0] = msg.linear_acceleration.x;
    a[1] = msg.linear_acceleration.y;
    a[2] = msg.linear_acceleration.z;
  }

  ros::Time t;        // timestamp
  Eigen::Vector3d w;  // gyro measurement
  Eigen::Vector3d a;  // accelerometer measurement
};

/**
 * @brief Struct representing the changes between imu states
 */
struct Delta {
  ros::Duration t;
  Eigen::Quaterniond q;
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Matrix<double, 15, 15> cov;  // ordered in q, p, v, bg, ba
  Eigen::Matrix<double, 15, 15> sqrt_inv_cov;
};

/**
 * @brief Represents the jacobian
 */
struct Jacobian {
  Eigen::Matrix3d dq_dbg;
  Eigen::Matrix3d dp_dbg;
  Eigen::Matrix3d dp_dba;
  Eigen::Matrix3d dv_dbg;
  Eigen::Matrix3d dv_dba;
};

/**
 * @brief this class contains the preintegration math and functions required, it
 * is used with ImuPreintegration for creating constraints and estimating states
 */
class PreIntegrator {
 public:
  /**
   * @brief Default Constructor
   */
  PreIntegrator() = default;

  /**
   * @brief Resets the preintegrator to a 0 state
   */
  void Reset();

  /**
   * @brief Increments current state by the incoming imu data
   * @param dt time difference to increment forward to
   * @param data given imu data
   * @param bg current gyroscope bias estimate
   * @param ba current accelerometer bias estimate
   * @param compute_jacobian optionally compute the jacobian
   * @param compute_covariance optionally compute the covariance
   */
  void Increment(ros::Duration dt, const IMUData& data,
                 const Eigen::Vector3d& bg, const Eigen::Vector3d& ba,
                 bool compute_jacobian, bool compute_covariance);

  /**
   * @brief IMU Integration Function, integrate current measurements to a
   * timestamp
   * @param t timestamp to integrate to
   * @param bg current gyroscope bias estimate
   * @param ba current accelerometer bias estimate
   * @param compute_jacobian optionally compute the jacobian
   * @param compute_covariance optionally compute the covariance
   */
  bool Integrate(ros::Time t, const Eigen::Vector3d& bg,
                 const Eigen::Vector3d& ba, bool compute_jacobian,
                 bool compute_covariance);

  /**
   * @brief Computes the square-root, inverted information matrix from the
   * covariance matrix calculated during preintegration
   */
  void ComputeSqrtInvCov();

  double cov_tol_{1e-9};  // tolarance on zero covariance matrix

  Eigen::Matrix3d cov_w;  // continuous noise covariance
  Eigen::Matrix3d cov_a;
  Eigen::Matrix3d cov_bg;  // continuous random walk noise covariance
  Eigen::Matrix3d cov_ba;

  Delta delta;
  Jacobian jacobian;
  std::vector<IMUData> data;  // vector of imu data (buffer)
};

}  // namespace bs_common
