#pragma once
#include <beam_common/preintegrator.h>
#include <beam_utils/utils.h>

namespace beam_models { namespace camera_to_camera {

/**
 * @brief this struct represents a single frame, which contains a pose,
 * timestamp and its preintegrator (which contains all the imu data since the
 * previous frame)
 */
struct Frame {
  ros::Time t;
  Eigen::Vector3d p;
  Eigen::Quaterniond q;
  beam_common::PreIntegrator preint;
};

/**
 * @brief this class can be used to estimate imu gyroscope bias, accelerometer
 * bias, gravity and scale given an input of estimated frame poses as well as
 * their preintegrator objects
 */
class IMUInitializer {
public:
  /**
   * @brief Custom Constructor
   * @param frames list of frames to use for estimation
   */
  IMUInitializer(
      const std::vector<beam_models::camera_to_camera::Frame>& frames);

  /**
   * @brief Solves for the gyroscope bias
   */
  void SolveGyroBias();

  /**
   * @brief Solves for acceleromater bias (gravity must be estimated before
   * calling this)
   */
  void SolveAccelBias();

  /**
   * @brief Solves for Gravity and Scale factor (scale factor only used if
   * frames are in arbitrary scale)
   */
  void SolveGravityAndScale();

  /**
   * @brief Refines the previously estimated gravity and scale factor
   */
  void RefineGravityAndScale();

  /**
   * @brief Read-only access to the gyroscope bias
   */
  const Eigen::Vector3d& GetGyroBias();

  /**
   * @brief Read-only access to the accelerometer bias
   */
  const Eigen::Vector3d& GetAccelBias();

  /**
   * @brief Read-only access to the gravity vector
   */
  const Eigen::Vector3d& GetGravity();

  /**
   * @brief Read-only access to the scale factor
   */
  const double& GetScale();

private:
  /**
   * @brief Integrates each frame using current bias estimates
   */
  void Integrate();

  Eigen::Matrix<double, 3, 2> s2_tangential_basis(const Eigen::Vector3d& x) {
    int d = 0;
    for (int i = 1; i < 3; ++i) {
      if (std::abs(x[i]) > std::abs(x[d])) d = i;
    }
    Eigen::Vector3d b1 =
        x.cross(Eigen::Vector3d::Unit((d + 1) % 3)).normalized();
    Eigen::Vector3d b2 = x.cross(b1).normalized();
    return (Eigen::Matrix<double, 3, 2>() << b1, b2).finished();
  }

  std::vector<beam_models::camera_to_camera::Frame> frames_;
  Eigen::Vector3d bg_;
  Eigen::Vector3d ba_;
  Eigen::Vector3d gravity_;
  double scale_;
};

}} // namespace beam_models::camera_to_camera
