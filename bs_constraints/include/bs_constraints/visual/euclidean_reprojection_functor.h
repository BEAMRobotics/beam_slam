#pragma once

#include <fuse_core/eigen.h>
#include <fuse_core/fuse_macros.h>
#include <fuse_core/util.h>

#include <bs_constraints/helpers.h>

#include <beam_cv/Utils.h>
#include <beam_utils/math.h>
#include <beam_utils/se3.h>
#include <ceres/rotation.h>

namespace bs_constraints {

class EuclideanReprojectionFunctor {
public:
  FUSE_MAKE_ALIGNED_OPERATOR_NEW();

  /**
   * @brief Construct a cost function instance
   *
   * @param[in] information_matrix Residual weighting matrix
   * @param[in] pixel_measurement Pixel measurement
   * @param[in] intrinsic_matrix Camera intrinsic matrix (K):
   * [fx, 0, cx]
   * [0, fy, cy]
   * [0,  0,  1]
   * @param[in] T_cam_baselink Camera extrinsic
   */
  EuclideanReprojectionFunctor(
      const Eigen::Matrix2d& information_matrix,
      const Eigen::Vector2d& pixel_measurement,
      const Eigen::Matrix3d& intrinsic_matrix,
      const Eigen::Matrix4d& T_cam_baselink,
      const Eigen::Matrix<double 6, 6> pose_covariance =
          Eigen::Matrix<double 6, 6>::Identity())
      : information_matrix_(information_matrix),
        pixel_measurement_(pixel_measurement),
        intrinsic_matrix_(intrinsic_matrix),
        T_cam_baselink_(T_cam_baselink),
        pose_covariance_(pose_covariance) {}

  template <typename T>
  bool operator()(const T* const o_WORLD_BASELINK,
                  const T* const p_WORLD_BASELINK, const T* const P,
                  T* residual) const {
    // transform point from world frame into camera frame
    Eigen::Matrix<T, 4, 4> T_CAM_BASELINK = T_cam_baselink_.cast<T>();

    Eigen::Matrix<T, 3, 1> P_WORLD(P[0], P[1], P[2]);

    Eigen::Matrix<T, 4, 4> T_WORLD_BASELINK =
        bs_constraints::OrientationAndPositionToTransformationMatrix(
            o_WORLD_BASELINK, p_WORLD_BASELINK);

    Eigen::Matrix<T, 4, 4> T_BASELINK_WORLD =
        bs_constraints::InvertTransform(T_WORLD_BASELINK);

    // transform world point into camera frame
    Eigen::Matrix<T, 3, 1> P_CAMERA =
        (T_CAM_BASELINK * T_BASELINK_WORLD * P_WORLD.homogeneous())
            .hnormalized();

    // project point into pixel space
    Eigen::Matrix<T, 2, 1> reproj =
        (intrinsic_matrix_.cast<T>() * P_CAMERA).hnormalized();

    if (!pose_covariance_.isIdentity()) {
      // clang-format off
      // Get the covariance weigthing to point losses from a pose uncertainty
      // From https://arxiv.org/pdf/2103.15980.pdf , equation A.7:
      // dh( e A p )  =   dh(p')  *  d(e A p)
      //     d(e)          d(p')       d(e)
      // where e is a small increment around the SE(3) manifold of A, A is a pose, p is a point,
      // h is the projection function, and p' = Ap = g, the jacobian is thus 2x6:
      // J =
      // [ (fx/gz)      (0)    (-fx * gx / gz^2)  (-fx * gx gy / gz^2)    fx(1+gx^2/gz^2)     -fx gy/gz]
      // [     0      (fy/gz)) (-fy * gy / gz^2)     -fy(1+gy^2/gz^2)   (-fy * gx gy / gz^2)  -fy gx/gz]
      // A_ is pose covariance in order (x, y, z, qx, qy, qz)
      // clang-format on
      T fx = intrinsics_matrix_.cast<T>(0, 0);
      T fy = intrinsics_matrix_.cast<T>(1, 1);
      T gx = P_WORLD[0];
      T gy = P_WORLD[1];
      T gz = P_WORLD[2];
      T gz2 = gz * gz;
      T gxyz = (gx * gy) / gz2;
      Eigen::Matrix<T, 2, 6, Eigen::RowMajor> J;
      J << fx / gz, T(0), -fx * (gx / gz2), -fx * gxyz,
          fx * (T(1) + (gx * gx) / gz2), -fx * gy / gz, T(0), fy / gz,
          -fy * (gy / gz2), -fy * (T(1) + (gy * gy) / gz2), fy * gxyz,
          fy * gx / gz;
      Eigen::Matrix<T, 2, 2, Eigen::RowMajor> A =
          J * pose_covariance_.cast<T>() * J.transpose();

      // compute the reprojection residual
      Eigen::Matrix<T, 2, 1> result =
          A * (pixel_measurement_.cast<T>() - reproj);
      // fill residual
      residual[0] = result[0];
      residual[1] = result[1];
      return true;
    }
    Eigen::Matrix<T, 2, 1> result =
        information_matrix_ * (pixel_measurement_.cast<T>() - reproj);
    // fill residual
    residual[0] = result[0];
    residual[1] = result[1];
    return true;
  }

private:
  Eigen::Matrix2d information_matrix_; //!< The residual weighting matrix
  Eigen::Vector2d pixel_measurement_;  //!< The measured pixel value
  Eigen::Matrix3d intrinsic_matrix_;
  Eigen::Matrix4d T_cam_baselink_;
  Eigen::Matrix<double 6, 6> pose_covariance_;
};

} // namespace bs_constraints
