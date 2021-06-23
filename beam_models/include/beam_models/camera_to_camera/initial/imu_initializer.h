#pragma once

#include <beam_models/camera_to_camera/initial/frame.h>
#include <beam_utils/utils.h>

namespace beam_models { namespace camera_to_camera {

class IMUInitializer {
public:
  IMUInitializer(
      const std::vector<beam_models::camera_to_camera::Frame>& frames);

  void Integrate();

  void SolveGyroBias();

  void SolveAccelBias();

  void SolveGravityAndScale();

  void RefineGravityAndScale();

  const Eigen::Vector3d& bg();

  const Eigen::Vector3d& ba();

  const Eigen::Vector3d& gravity();

  const double& scale();

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

private:
  std::vector<beam_models::camera_to_camera::Frame> frames_;
  Eigen::Vector3d bg_;
  Eigen::Vector3d ba_;
  Eigen::Vector3d gravity_;
  double scale_;
};

}} // namespace beam_models::camera_to_camera
