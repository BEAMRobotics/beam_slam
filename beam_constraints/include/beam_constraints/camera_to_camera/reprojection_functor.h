#ifndef FUSE_MODELS_VISUAL_COST_FUNCTOR_H
#define FUSE_MODELS_VISUAL_COST_FUNCTOR_H

#include <fuse_core/eigen.h>
#include <fuse_core/macros.h>
#include <fuse_core/util.h>

#include <beam_calibration/CameraModel.h>
#include <beam_optimization/CamPoseReprojectionCost.h>
#include <beam_utils/math.h>

#include <ceres/autodiff_cost_function.h>
#include <ceres/cost_function_to_functor.h>
#include <ceres/numeric_diff_cost_function.h>
#include <ceres/rotation.h>

template <class T>
using opt = beam::optional<T>;

namespace fuse_constraints {

class ReprojectionFunctor {
public:
  FUSE_MAKE_ALIGNED_OPERATOR_NEW();

  /**
   * @brief Construct a cost function instance
   *
   * @param[in] A The residual weighting matrix, Recommended: 2x2 identity
   * @param[in] pixel_measurement The pixel location of feature in the image
   * @param[in] cam_model The camera intrinsics for projection
   */
  ReprojectionFunctor(
      const fuse_core::Matrix2d& A, const Eigen::Vector2d& pixel_measurement,
      const std::shared_ptr<beam_calibration::CameraModel> cam_model,
      const Eigen::Matrix4d& T_imu_cam)
      : A_(A), pixel_measurement_(pixel_measurement), cam_model_(cam_model) {
    compute_projection.reset(new ceres::CostFunctionToFunctor<2, 3>(
        new ceres::NumericDiffCostFunction<beam_optimization::CameraProjectionFunctor,
                                           ceres::CENTRAL, 2, 3>(
            new beam_optimization::CameraProjectionFunctor(cam_model_, pixel_measurement_))));
    beam::TransformMatrixToQuaternionAndTranslation(T_imu_cam.inverse(),
                                                    Q_cam_imu_, t_cam_imu_);
  }

  template <typename T>
  bool operator()(const T* const R_WORLD_IMU, const T* const t_WORLD_IMU,
                  const T* const P_WORLD, T* residual) const {
    T R_IMU_WORLD[4];
    R_IMU_WORLD[0] = R_WORLD_IMU[0];
    R_IMU_WORLD[1] = -R_WORLD_IMU[1];
    R_IMU_WORLD[2] = -R_WORLD_IMU[2];
    R_IMU_WORLD[3] = -R_WORLD_IMU[3];
    // rotate and translate point (world to imu frame)
    T P_IMU[3];
    ceres::QuaternionRotatePoint(R_IMU_WORLD, P_WORLD, P_IMU);
    T Rt[3];
    ceres::QuaternionRotatePoint(R_IMU_WORLD, t_WORLD_IMU, Rt);
    P_IMU[0] -= Rt[0];
    P_IMU[1] -= Rt[1];
    P_IMU[2] -= Rt[2];

    // T P_IMU[3];
    // ceres::QuaternionRotatePoint(R_IMU_WORLD, P_WORLD, P_IMU);
    // P_IMU[0] += t_WORLD_IMU[0];
    // P_IMU[1] += t_WORLD_IMU[1];
    // P_IMU[2] += t_WORLD_IMU[2];

    // extrinsic transform (imu to camera)
    T P_CAMERA[3];
    ceres::QuaternionRotatePoint(Q_cam_imu_.cast<T>().coeffs().data(), P_IMU,
                                 P_CAMERA);
    P_CAMERA[0] += t_cam_imu_.cast<T>()[0];
    P_CAMERA[1] += t_cam_imu_.cast<T>()[1];
    P_CAMERA[2] += t_cam_imu_.cast<T>()[2];

    const T* P_CAMERA_const = &(P_CAMERA[0]);

    T pixel_projected[2];
    (*compute_projection)(P_CAMERA_const, &(pixel_projected[0]));

    residual[0] = pixel_measurement_.cast<T>()[0] - pixel_projected[0];
    residual[1] = pixel_measurement_.cast<T>()[1] - pixel_projected[1];
    return true;
  }

private:
  fuse_core::Matrix2d A_;
  Eigen::Vector2d pixel_measurement_;
  std::shared_ptr<beam_calibration::CameraModel> cam_model_;
  std::unique_ptr<ceres::CostFunctionToFunctor<2, 3>> compute_projection;
  Eigen::Quaterniond Q_cam_imu_;
  Eigen::Vector3d t_cam_imu_;
};

} // namespace fuse_constraints

#endif // FUSE_MODELS_VISUAL_COST_FUNCTOR_H
