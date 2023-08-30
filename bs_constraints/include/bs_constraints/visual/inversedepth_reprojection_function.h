#ifndef FUSE_MODELS_InverseDepth_REPROJECTION_FUNCTION_H
#define FUSE_MODELS_InverseDepth_REPROJECTION_FUNCTION_H

#include <ceres/sized_cost_function.h>

#include <beam_utils/math.h>
#include <beam_utils/se3.h>
#include <bs_constraints/jacobians.h>
#include <fuse_core/fuse_macros.h>
#include <fuse_core/util.h>

#include <ceres/rotation.h>

constexpr double EPSILON = 1e-8;

auto point_transformation = [](const auto& q_coeffs, const auto& t,
                               const auto& P) {
  // get R from q_coeffs
  Eigen::Quaterniond q(q_coeffs[0], q_coeffs[1], q_coeffs[2], q_coeffs[3]);
  Eigen::Matrix3d R = q.normalized().toRotationMatrix();
  Eigen::Matrix3d Ri = R.transpose();
  // invert R
  Eigen::Vector3d P_transformed = (Ri * P) - (Ri * t);
  return P_transformed;
};

namespace bs_constraints {

class InverseDepthReprojection : public ceres::SizedCostFunction<2, 4, 3, 3> {
public:
  FUSE_MAKE_ALIGNED_OPERATOR_NEW();

  InverseDepthReprojection(const Eigen::Matrix2d& A, const Eigen::Vector2d& b,
                        const Eigen::Matrix3d& intrinsic_matrix,
                        const Eigen::Matrix4d& T_cam_baselink)
      : A_(A),
        b_(b),
        intrinsic_matrix_(intrinsic_matrix),
        T_cam_baselink_(T_cam_baselink) {
    bearing_[0] = (b_[0] - intrinsic_matrix_(0, 2)) / intrinsic_matrix_(0, 0);
    bearing_[1] = (b_[1] - intrinsic_matrix_(1, 2)) / intrinsic_matrix_(1, 1);
    bearing_[2] = 1.0;
  }

  /**
   * @brief Evaluate the cost function. Used by the Ceres optimization engine.
   *
   * @param[in] parameters - Parameter blocks:
   *                         0 : R_WORLD_BASELINK (4d quaternion of robot)
   *                         1 : t_WORLD_BASELINK (3d position of robot)
   *                         2 : P_WORLD (3d position of landmark)
   * @param[out] residual - The computed residual (error)
   * @param[out] jacobians - Jacobians of the residuals wrt the parameters. Only
   * computed if not NULL, and only computed for the parameters where
   * jacobians[i] is not NULL.
   * @return The return value indicates whether the computation of the residuals
   * and/or jacobians was successful or not.
   */
  bool Evaluate(double const* const* parameters, double* residual,
                double** jacobians) const override {
    Eigen::Quaterniond q_WORLD_BASELINKa(parameters[0][0], parameters[0][1],
                                         parameters[0][2], parameters[0][3]);
    Eigen::Matrix3d R_WORLD_BASELINKa = q_WORLD_BASELINKa.toRotationMatrix();
    Eigen::Vector3d t_WORLD_BASELINKa(parameters[1][0], parameters[1][1],
                                      parameters[1][2]);
    Eigen::Quaterniond q_WORLD_BASELINKm(parameters[2][0], parameters[2][1],
                                         parameters[2][2], parameters[2][3]);
    Eigen::Matrix3d R_WORLD_BASELINKm = q_WORLD_BASELINKm.toRotationMatrix();
    Eigen::Vector3d t_WORLD_BASELINKm(parameters[3][0], parameters[3][1],
                                      parameters[3][2]);
    Eigen::Vector3d rho(parameters[4][0]);

    Eigen::Matrix4d T_WORLD_BASELINKa;
    beam::QuaternionAndTranslationToTransformMatrix(
        q_WORLD_BASELINKa, t_WORLD_BASELINKa, T_WORLD_BASELINKa);
    Eigen::Matrix4d T_WORLD_BASELINKm;
    beam::QuaternionAndTranslationToTransformMatrix(
        q_WORLD_BASELINKm, t_WORLD_BASELINKm, T_WORLD_BASELINKm);

    Eigen::Matrix4d T_BASELINK_CAM = beam::InvertTransform(T_cam_baselink_);
    Eigen::Matrix4d T_WORLD_CAMERAa = T_WORLD_BASELINKa * T_BASELINK_CAM;
    Eigen::Matrix4d T_WORLD_CAMERAm = T_WORLD_BASELINKm * T_BASELINK_CAM;

    Eigen::Matrix4d T_CAMERAm_CAMERAa =
        beam::InvertTransform(T_WORLD_CAMERAm) * T_WORLD_CAMERAa;

    const Eigen::Matrix<double, 3, 4> projection_matrix =
        intrinsic_matrix_ * T_CAMERAm_CAMERAa.block<3, 4>(0, 0);

    // compute the inverse depth and bearing vector (mx, my, 1, rho)
    Eigen::Vector4d idp;
    idp << bearing_, rho;

    // project into measurement image
    Eigen::Vector2d projected = (projection_matrix * idp).hnormalized();

    // compute error
    Eigen::Vector2d E = A_ * (b_ - projected);
    residual[0] = E[0];
    residual[1] = E[1];

    // compute jacobians
    if (jacobians) {
      if (jacobians[0]) {
        // d(E)/d(R_WORLD_BASELINKa) = d(E)/d(measurement_t_point) *
        // d(measurement_t_point)/d(R_CAMERAm_CAMERAa) *
        // d(R_CAMERAm_CAMERAa)/d(R_WORLD_CAMERAa) *
        // d(R_WORLD_CAMERAa)/d(R_WORLD_BASELINKa)
      }

      if (jacobians[1]) {}

      if (jacobians[2]) {}

      if (jacobians[3]) {}

      if (jacobians[4]) {}
    }
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

#endif // FUSE_MODELS_InverseDepth_REPROJECTION_FUNCTION_H
