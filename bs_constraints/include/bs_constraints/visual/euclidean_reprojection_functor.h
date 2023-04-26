#ifndef FUSE_MODELS_EUCLIDEAN_REPROJECTION_FUNCTOR_H
#define FUSE_MODELS_EUCLIDEAN_REPROJECTION_FUNCTOR_H

#include <fuse_core/eigen.h>
#include <fuse_core/fuse_macros.h>
#include <fuse_core/util.h>

#include <beam_utils/math.h>
#include <beam_utils/se3.h>
#include <bs_constraints/jacobians.h>

#include <ceres/autodiff_cost_function.h>
#include <ceres/cost_function_to_functor.h>
#include <ceres/numeric_diff_cost_function.h>
#include <ceres/rotation.h>

namespace fuse_constraints {

class EuclideanReprojectionFunctor {
public:
  FUSE_MAKE_ALIGNED_OPERATOR_NEW();

  /**
   * @brief Construct a cost function instance
   * @param A sqrt information matrix
   * @param b pixel measurement (undistorted)
   * @param intrinsic_matrix K matrix for camera
   * @param T_cam_baselink extrinsic from baselink to camera
   */
  EuclideanReprojectionFunctor(const Eigen::Matrix2d& A,
                               const Eigen::Vector2d& b,
                               const Eigen::Matrix3d& intrinsic_matrix,
                               const Eigen::Matrix4d& T_cam_baselink)
      : A_(A),
        b_(b),
        intrinsic_matrix_(intrinsic_matrix),
        T_cam_baselink_(T_cam_baselink) {}

  template <typename T>
  bool operator()(const double* const R_WORLD_BASELINK,
                  const double* const t_WORLD_BASELINK,
                  const double* const P_WORLD, double* residual,
                  double** jacobians) const {
    // get robot pose as a transformation matrix
    double R_vec[9];
    ceres::QuaternionToRotation(R_WORLD_BASELINK, R_vec);
    Eigen::Matrix3d R(R_vec);
    Eigen::Vector3d t(t_WORLD_BASELINK);
    Eigen::Matrix4d T_WORLD_BASELINK = Eigen::Matrix4d::Identity();
    T_WORLD_BASELINK.block<3, 3>(0, 0) = R;
    T_WORLD_BASELINK.block<3, 1>(0, 3) = t;

    // transform landmark into camera frame
    Eigen::Matrix4d T_CAMERA_WORLD =
        T_CAM_BASELINK * beam::InvertTransform(T_WORLD_BASELINK);
    Eigen::Vector3d P(P_WORLD);
    Eigen::Vector4d P_CAMERA = (T_CAMERA_WORLD * P.homogeneous()).hnormalized();

    // project into image space
    Eigen::Vector2d reprojection = (intrinsic_matrix_ * P_CAMERA).hnormalized();

    residual[0] = b_[0] - reprojection[0];
    residual[1] = b_[1] - reprojection[1];

    if (jacobians) {
      const auto d_E_d_P_CAMERA =
          DImageProjectionDPoint(intrinsic_matrix_, P_CAMERA);
      const auto d_P_CAMERA_d_T_CAMERA_WORLD =
          DPointTransformationDTransform(T_CAMERA_WORLD, P_CAMERA);

      if (jacobians[0] || jacobians[1]) {
        // compute d(E)/d(T_WORLD_BASELINK) = d(E)/d(P_CAMERA) *
        // d(P_CAMERA)/d(T_CAMERA_WORLD) *
        // d(T_CAMERA_WORLD)/d(T_BASELINK_WORLD) *
        // d(T_BASELINK_WORLD)/d(T_WORLD_BASELINK)
        const auto d_T_BASELINK_WORLD_d_T_WORLD_BASELINK =
            DInverseTransformDTransform(T_WORLD_BASELINK);
        const auto d_T_CAMERA_WORLD_d_T_BASELINK_WORLD =
            DTransformCompositionDRightTransform();

        // jacobian for full transform
        const auto d_E_d_T_WORLD_BASELINK =
            d_E_d_P_CAMERA * d_P_CAMERA_d_T_CAMERA_WORLD *
            d_T_CAMERA_WORLD_d_T_BASELINK_WORLD *
            d_T_BASELINK_WORLD_d_T_WORLD_BASELINK;

        // break jacobian into parts
        const auto d_E_d_R_WORLD_BASELINK =
            d_E_d_T_WORLD_BASELINK.block<3, 3>(0, 0);
        const auto d_E_d_t_WORLD_BASELINK =
            d_E_d_T_WORLD_BASELINK.block<3, 3>(3, 3);
        jacobians[0] = d_E_d_R_WORLD_BASELINK.data();
        jacobians[1] = d_E_d_t_WORLD_BASELINK.data();
      }

      if (jacobians[2]) {
        // compute d(E)/d(P_WORLD) = d(E)/d(P_CAMERA) * d(P_CAMERA)/d(P_WORLD)
        const auto d_P_CAMERA_D_P_WORLD =
            DPointTransformationDPoint(T_CAMERA_WORLD);
        const auto d_E_D_P_WORLD = d_E_d_P_CAMERA * d_P_CAMERA_D_P_WORLD;
        jacobians[2] = d_E_D_P_WORLD.data();
      }
    }

    return true;
  }

private:
  Eigen::Matrix2d A_;
  Eigen::Vector2d b_;
  Eigen::Matrix3d intrinsic_matrix_;
  Eigen::Matrix4d T_cam_baselink_;
};

} // namespace fuse_constraints

#endif // FUSE_MODELS_EUCLIDEAN_REPROJECTION_FUNCTOR_H
