#pragma once

#include <fuse_core/eigen.h>
#include <fuse_core/fuse_macros.h>
#include <fuse_core/util.h>

#include <beam_cv/Utils.h>
#include <beam_utils/math.h>
#include <beam_utils/se3.h>
#include <ceres/rotation.h>

namespace bs_constraints {

class InverseDepthReprojectionFunctorUnary {
public:
  FUSE_MAKE_ALIGNED_OPERATOR_NEW();

  /**
   * @brief Construct a cost function instance
   *
   * @param[in] pixel_measurement The pixel location of feature in the image
   * @param[in] cam_model The camera intrinsics for projection
   */
  InverseDepthReprojectionFunctorUnary(const Eigen::Matrix2d& A,
                                       const Eigen::Vector2d& b,
                                       const Eigen::Matrix3d& intrinsic_matrix,
                                       const Eigen::Matrix4d& T_cam_baselink,
                                       const Eigen::Vector3d& bearing)
      : A_(A),
        b_(b),
        intrinsic_matrix_(intrinsic_matrix),
        T_cam_baselink_(T_cam_baselink),
        bearing_(bearing) {}

  template <typename T>
  bool operator()(const T* const q_anchor, const T* const t_anchor,
                  const T* const rho, T* residual) const {
    // get relative pose between anchor and measurement
    Eigen::Matrix<T, 4, 4> T_CAMERAm_CAMERAa =
        Eigen::Matrix<T, 4, 4>::Identity();

    // create projection matrix
    const Eigen::Matrix<T, 3, 4> projection_matrix =
        intrinsic_matrix_.cast<T>() * T_CAMERAm_CAMERAa.block(0, 0, 3, 4);

    // compute the inverse depth and bearing vector (mx, my, 1, rho)
    Eigen::Matrix<T, 4, 1> InverseDepth;
    InverseDepth << bearing_.cast<T>(), rho;

    // project into measurement image
    Eigen::Matrix<T, 2, 1> reproj =
        (projection_matrix * InverseDepth).hnormalized();

    Eigen::Matrix<T, 2, 1> E = A_.cast<T>() * (b_.cast<T>() - reproj);
    residual[0] = E[0];
    residual[1] = E[1];

    return true;
  }

private:
  Eigen::Matrix2d A_; //!< The residual weighting matrix
  Eigen::Vector2d b_; //!< The measured pixel value
  Eigen::Vector3d bearing_;
  Eigen::Matrix3d intrinsic_matrix_;
  Eigen::Matrix4d T_cam_baselink_;
};

} // namespace bs_constraints
