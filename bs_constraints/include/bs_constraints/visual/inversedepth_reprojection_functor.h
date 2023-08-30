#ifndef FUSE_MODELS_VISUAL_COST_FUNCTOR_H
#define FUSE_MODELS_VISUAL_COST_FUNCTOR_H

#include <fuse_core/eigen.h>
#include <fuse_core/fuse_macros.h>
#include <fuse_core/util.h>

#include <beam_calibration/CameraModel.h>
#include <beam_cv/Utils.h>
#include <beam_utils/math.h>
#include <beam_utils/se3.h>
#include <ceres/rotation.h>

namespace bs_constraints {

class InverseDepthReprojectionFunctor {
public:
  FUSE_MAKE_ALIGNED_OPERATOR_NEW();

  /**
   * @brief Construct a cost function instance
   *
   * @param[in] pixel_measurement The pixel location of feature in the image
   * @param[in] cam_model The camera intrinsics for projection
   */
  InverseDepthReprojectionFunctor(const Eigen::Matrix2d& A,
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
                  const T* const q_measurement, const T* const t_measurement,
                  const T* const rho, T* residual) const {
    // get extrinsic
    Eigen::Matrix<T, 4, 4> T_BASELINK_CAM =
        beam::InvertTransform(T_cam_baselink_).cast<T>();
    Eigen::Matrix<T, 4, 4> T_CAM_BASELINK = T_cam_baselink_.cast<T>();

    // get anchor pose as 4x4 matrix
    T Ra[9];
    ceres::QuaternionToRotation(q_anchor, Ra);
    Eigen::Matrix<T, 3, 3> R_WORLD_BASELINKa;
    R_WORLD_BASELINKa << Ra[0], Ra[1], Ra[2], Ra[3], Ra[4], Ra[5], Ra[6], Ra[7],
        Ra[8];
    Eigen::Matrix<T, 3, 1> t_WORLD_BASELINKa(t_anchor[0], t_anchor[1],
                                             t_anchor[2]);
    Eigen::Matrix<T, 4, 4> T_WORLD_BASELINKa =
        Eigen::Matrix<T, 4, 4>::Identity();
    T_WORLD_BASELINKa.block(0, 0, 3, 3) = R_WORLD_BASELINKa;
    T_WORLD_BASELINKa.block(0, 3, 3, 1) = t_WORLD_BASELINKa;

    // get measurement pose as 4x4 matrix
    T Rm[9];
    ceres::QuaternionToRotation(q_measurement, Rm);
    Eigen::Matrix<T, 3, 3> R_WORLD_BASELINKm;
    R_WORLD_BASELINKm << Rm[0], Rm[1], Rm[2], Rm[3], Rm[4], Rm[5], Rm[6], Rm[7],
        Rm[8];
    Eigen::Matrix<T, 3, 1> t_WORLD_BASELINKm(t_measurement[0], t_measurement[1],
                                             t_measurement[2]);
    Eigen::Matrix<T, 4, 4> T_BASELINKm_WORLD =
        Eigen::Matrix<T, 4, 4>::Identity();
    T_BASELINKm_WORLD.block(0, 0, 3, 3) = R_WORLD_BASELINKm.transpose();
    T_BASELINKm_WORLD.block(0, 3, 3, 1) =
        -R_WORLD_BASELINKm.transpose() * t_WORLD_BASELINKm;

    // get poses wrt camera frame
    Eigen::Matrix<T, 4, 4> T_WORLD_CAMERAa = T_WORLD_BASELINKa * T_BASELINK_CAM;
    Eigen::Matrix<T, 4, 4> T_CAMERAm_WORLD = T_CAM_BASELINK * T_BASELINKm_WORLD;

    // get relative pose between anchor and measurement
    Eigen::Matrix<T, 4, 4> T_CAMERAm_CAMERAa =
        T_CAMERAm_WORLD * T_WORLD_CAMERAa;

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

#endif // FUSE_MODELS_VISUAL_COST_FUNCTOR_H