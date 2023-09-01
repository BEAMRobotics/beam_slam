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
  EuclideanReprojectionFunctor(const Eigen::Matrix2d& information_matrix,
                               const Eigen::Vector2d& pixel_measurement,
                               const Eigen::Matrix3d& intrinsic_matrix,
                               const Eigen::Matrix4d& T_cam_baselink)
      : information_matrix_(information_matrix),
        pixel_measurement_(pixel_measurement),
        intrinsic_matrix_(intrinsic_matrix),
        T_cam_baselink_(T_cam_baselink) {}

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

    // compute the reprojection residual
    Eigen::Matrix<T, 2, 1> result;
    result =
        information_matrix_.cast<T>() * (pixel_measurement_.cast<T>() - reproj);

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
};

} // namespace bs_constraints
