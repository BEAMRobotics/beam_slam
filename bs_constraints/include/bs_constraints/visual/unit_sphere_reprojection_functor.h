#ifndef FUSE_MODELS_US_VISUAL_COST_FUNCTOR_H
#define FUSE_MODELS_US_VISUAL_COST_FUNCTOR_H

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

namespace fuse_constraints {

class UnitSphereReprojectionFunctor {
public:
  FUSE_MAKE_ALIGNED_OPERATOR_NEW();

  /**
   * @brief Construct a cost function instance
   *
   * @param[in] pixel_measurement The pixel location of feature in the image
   * @param[in] cam_model The camera intrinsics for projection
   */
  UnitSphereReprojectionFunctor(
      const Eigen::Vector2d& pixel_measurement,
      const std::shared_ptr<beam_calibration::CameraModel> cam_model,
      const Eigen::Matrix4d& T_cam_baselink)
      : pixel_measurement_(pixel_measurement),
        cam_model_(cam_model),
        T_cam_baselink_(T_cam_baselink) {
    // get pixel in the unit sphere
    Eigen::Vector2i pixel_i = pixel_measurement.cast<int>();
    cam_model_->BackProject(pixel_i, unit_sphere_pixel_);

    // compute tanget base of the measurement
    Eigen::Vector3d b1, b2;
    Eigen::Vector3d a = unit_sphere_pixel_.normalized();
    Eigen::Vector3d tmp(0, 0, 1);
    if (a == tmp) tmp << 1, 0, 0;
    b1 = (tmp - a * (a.transpose() * tmp)).normalized();
    b2 = a.cross(b1);
    tangent_base_.block<1, 3>(0, 0) = b1.transpose();
    tangent_base_.block<1, 3>(1, 0) = b2.transpose();

    // compute sqrt information matrix
    sqrt_info_ = Eigen::Matrix2d::Identity();
    sqrt_info_(0, 0) = cam_model_->GetIntrinsics()[0] / 1.5;
    sqrt_info_(1, 1) = cam_model_->GetIntrinsics()[1] / 1.5;
  }

  template <typename T>
  bool operator()(const T* const R_WORLD_BASELINK,
                  const T* const t_WORLD_BASELINK, const T* const P_WORLD,
                  T* residual) const {
    // transform point from world frame into camera frame
    Eigen::Matrix<T, 4, 4> T_CAM_BASELINK = T_cam_baselink_.cast<T>();

    T R_WORLD_BASELINK_mat[9];
    ceres::QuaternionToRotation(R_WORLD_BASELINK, R_WORLD_BASELINK_mat);

    Eigen::Matrix<T, 4, 4> T_WORLD_BASELINK;
    T_WORLD_BASELINK(0, 0) = R_WORLD_BASELINK_mat[0];
    T_WORLD_BASELINK(0, 1) = R_WORLD_BASELINK_mat[1];
    T_WORLD_BASELINK(0, 2) = R_WORLD_BASELINK_mat[2];
    T_WORLD_BASELINK(0, 3) = t_WORLD_BASELINK[0];
    T_WORLD_BASELINK(1, 0) = R_WORLD_BASELINK_mat[3];
    T_WORLD_BASELINK(1, 1) = R_WORLD_BASELINK_mat[4];
    T_WORLD_BASELINK(1, 2) = R_WORLD_BASELINK_mat[5];
    T_WORLD_BASELINK(1, 3) = t_WORLD_BASELINK[1];
    T_WORLD_BASELINK(2, 0) = R_WORLD_BASELINK_mat[6];
    T_WORLD_BASELINK(2, 1) = R_WORLD_BASELINK_mat[7];
    T_WORLD_BASELINK(2, 2) = R_WORLD_BASELINK_mat[8];
    T_WORLD_BASELINK(2, 3) = t_WORLD_BASELINK[2];
    T_WORLD_BASELINK(3, 0) = (T)0;
    T_WORLD_BASELINK(3, 1) = (T)0;
    T_WORLD_BASELINK(3, 2) = (T)0;
    T_WORLD_BASELINK(3, 3) = (T)1;

    Eigen::Matrix<T, 4, 1> P_WORLD_h;
    P_WORLD_h[0] = P_WORLD[0];
    P_WORLD_h[1] = P_WORLD[1];
    P_WORLD_h[2] = P_WORLD[2];
    P_WORLD_h[3] = (T)1;

    Eigen::Matrix<T, 4, 1> P_BASELINK_h =
        T_WORLD_BASELINK.inverse() * P_WORLD_h;
    Eigen::Matrix<T, 3, 1> P_CAM =
        (T_CAM_BASELINK * P_BASELINK_h).hnormalized();

    // compute the residual on the unit sphere
    Eigen::Matrix<T, 2, 1> result =
        tangent_base_.cast<T>() *
        (P_CAM.normalized() - unit_sphere_pixel_.cast<T>().normalized());

    // apply sqrt information matrix
    result = sqrt_info_.cast<T>() * result;

    // fill residual
    residual[0] = result[0];
    residual[1] = result[1];

    return true;
  }

private:
  Eigen::Matrix2d sqrt_info_;
  Eigen::Vector2d pixel_measurement_;
  Eigen::Vector3d unit_sphere_pixel_;
  std::shared_ptr<beam_calibration::CameraModel> cam_model_;
  Eigen::Matrix4d T_cam_baselink_;
  Eigen::Matrix<double, 2, 3> tangent_base_;
  bool in_domain_{true};
};

} // namespace fuse_constraints

#endif // FUSE_MODELS_US_VISUAL_COST_FUNCTOR_H
