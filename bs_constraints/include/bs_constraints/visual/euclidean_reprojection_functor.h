#pragma once

#include <fuse_core/eigen.h>
#include <fuse_core/fuse_macros.h>
#include <fuse_core/util.h>

#include <beam_calibration/CameraModel.h>
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
   * @param[in] pixel_measurement The pixel location of feature in the image
   * @param[in] cam_model The camera intrinsics for projection
   */
  EuclideanReprojectionFunctor(const Eigen::Matrix2d& A, const Eigen::Vector2d& b,
                      const Eigen::Matrix3d& intrinsic_matrix,
                      const Eigen::Matrix4d& T_cam_baselink)
      : A_(A),
        b_(b),
        intrinsic_matrix_(intrinsic_matrix),
        T_cam_baselink_(T_cam_baselink) {}

  template <typename T>
  bool operator()(const T* const q, const T* const t, const T* const P,
                  T* residual) const {
    // transform point from world frame into camera frame
    Eigen::Matrix<T, 4, 4> T_CAM_BASELINK = T_cam_baselink_.cast<T>();

    T R[9];
    ceres::QuaternionToRotation(q, R);

    Eigen::Matrix<T, 3, 3> R_WORLD_BASELINK;
    R_WORLD_BASELINK << R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8];
    Eigen::Matrix<T, 3, 1> t_WORLD_BASELINK(t[0], t[1], t[2]);
    Eigen::Matrix<T, 3, 1> P_WORLD(P[0], P[1], P[2]);

    Eigen::Matrix<T, 4, 4> T_WORLD_BASELINK =
        Eigen::Matrix<T, 4, 4>::Identity();
    T_WORLD_BASELINK.block(0, 0, 3, 3) = R_WORLD_BASELINK;
    T_WORLD_BASELINK.block(0, 3, 3, 1) = t_WORLD_BASELINK;

    Eigen::Matrix<T, 4, 4> T_BASELINK_WORLD =
        Eigen::Matrix<T, 4, 4>::Identity();
    T_BASELINK_WORLD.block(0, 0, 3, 3) = R_WORLD_BASELINK.transpose();
    T_BASELINK_WORLD.block(0, 3, 3, 1) =
        -R_WORLD_BASELINK.transpose() * t_WORLD_BASELINK;

    // transform world point into camera frame
    Eigen::Matrix<T, 3, 1> P_CAMERA =
        (T_CAM_BASELINK * T_BASELINK_WORLD * P_WORLD.homogeneous())
            .hnormalized();

    // project point into pixel space
    Eigen::Matrix<T, 2, 1> reproj =
        (intrinsic_matrix_.cast<T>() * P_CAMERA).hnormalized();

    // compute the reprojection residual
    Eigen::Matrix<T, 2, 1> result;
    result = A_.cast<T>() * (b_.cast<T>() - reproj);

    // fill residual
    residual[0] = result[0];
    residual[1] = result[1];
    return true;
  }

private:
  Eigen::Matrix2d A_; //!< The residual weighting matrix
  Eigen::Vector2d b_; //!< The measured pixel value
  Eigen::Matrix3d intrinsic_matrix_;
  Eigen::Matrix4d T_cam_baselink_;
};

} // namespace bs_constraints
