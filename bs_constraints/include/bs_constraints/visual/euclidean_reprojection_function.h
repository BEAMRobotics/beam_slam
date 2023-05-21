#ifndef FUSE_MODELS_EUCLIDEAN_REPROJECTION_FUNCTION_H
#define FUSE_MODELS_EUCLIDEAN_REPROJECTION_FUNCTION_H

#include <ceres/sized_cost_function.h>

#include <beam_utils/math.h>
#include <beam_utils/se3.h>
#include <bs_constraints/jacobians.h>
#include <fuse_core/fuse_macros.h>
#include <fuse_core/util.h>

#include <ceres/rotation.h>

namespace bs_constraints {

class EuclideanReprojection : public ceres::SizedCostFunction<2, 4, 3, 3> {
public:
  FUSE_MAKE_ALIGNED_OPERATOR_NEW();

  EuclideanReprojection(const Eigen::Matrix2d& A, const Eigen::Vector2d& b,
                        const Eigen::Matrix3d& intrinsic_matrix,
                        const Eigen::Matrix4d& T_cam_baselink)
      : A_(A),
        b_(b),
        intrinsic_matrix_(intrinsic_matrix),
        T_cam_baselink_(T_cam_baselink) {}

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
    Eigen::Quaterniond q_WORLD_BASELINK(parameters[0][0], parameters[0][1],
                                        parameters[0][2], parameters[0][3]);
    Eigen::Matrix3d R_WORLD_BASELINK = q_WORLD_BASELINK.toRotationMatrix();
    Eigen::Vector3d t_WORLD_BASELINK(parameters[1][0], parameters[1][1],
                                     parameters[1][2]);
    Eigen::Vector3d P_WORLD(parameters[2][0], parameters[2][1],
                            parameters[2][2]);

    // get robot pose as a transformation matrix
    Eigen::Matrix4d T_WORLD_BASELINK = Eigen::Matrix4d::Identity();
    T_WORLD_BASELINK.block<3, 3>(0, 0) = q_WORLD_BASELINK.toRotationMatrix();
    T_WORLD_BASELINK.block<3, 1>(0, 3) = t_WORLD_BASELINK;
    Eigen::Matrix4d T_BASELINK_WORLD = T_WORLD_BASELINK;

    Eigen::Matrix3d R_CAM_BASELINK = T_cam_baselink_.block<3, 3>(0, 0);
    Eigen::Vector3d t_CAM_BASELINK = T_cam_baselink_.block<3, 1>(0, 3);
    Eigen::Matrix3d R_BASELINK_WORLD = R_WORLD_BASELINK.transpose();

    // 1. transform point into baselink frame
    auto P_BASELINK =
        (R_BASELINK_WORLD * P_WORLD) - (R_BASELINK_WORLD * t_WORLD_BASELINK);

    // 2. transform point into camera frame
    auto P_CAMERA = R_CAM_BASELINK * P_BASELINK + t_CAM_BASELINK;

    //  3. project into image space
    Eigen::Vector2d reprojection = (intrinsic_matrix_ * P_CAMERA).hnormalized();

    // compute weighted reprojection error
    Eigen::Vector2d E = A_ * (b_ - reprojection);
    residual[0] = E[0];
    residual[1] = E[1];

    // compute jacobians
    if (jacobians) {
      const auto d_E_d_P_CAMERA =
          DImageProjectionDPoint(intrinsic_matrix_, P_CAMERA);
      const auto d_P_CAMERA_d_P_BASELINK =
          DPointRotationDPoint(R_CAM_BASELINK, P_BASELINK);
      if (jacobians[0]) {
        // compute d(E)/d(R_WORLD_BASELINK) = d(E)/d(P_CAMERA) *
        // d(P_CAMERA)/d(P_BASELINK) *
        // d(P_BASELINK)/d(R_WORLD_BASELINK)
        const auto d_P_BASELINK_D_R_WORLD_BASELINK =
            DPointRotationDRotation(R_BASELINK_WORLD, P_WORLD) *
                DInverseRotationDRotation(R_WORLD_BASELINK) -
            DPointRotationDRotation(R_BASELINK_WORLD, t_WORLD_BASELINK) *
                DInverseRotationDRotation(R_WORLD_BASELINK);
        auto d_E_d_R_WORLD_BASELINK = d_E_d_P_CAMERA * d_P_CAMERA_d_P_BASELINK *
                                      d_P_BASELINK_D_R_WORLD_BASELINK;
        // lift to jacobian representation
        Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>>
            d_E_d_q_WORLD_BASELINK(jacobians[0]);

        // d_E_d_q_WORLD_BASELINK = Eigen::Matrix<double, 2, 4>::Zero();
        // d_E_d_q_WORLD_BASELINK.block<2, 3>(0, 0) = d_E_d_R_WORLD_BASELINK;

        d_E_d_q_WORLD_BASELINK =
            d_E_d_R_WORLD_BASELINK * LiftJacobian(q_WORLD_BASELINK);
        d_E_d_q_WORLD_BASELINK = -d_E_d_q_WORLD_BASELINK;
      }

      if (jacobians[1]) {
        // compute d(E)/d(R_WORLD_BASELINK) = d(E)/d(P_CAMERA) *
        // d(P_CAMERA)/d(P_BASELINK) *
        // d(P_BASELINK)/d(t_WORLD_BASELINK)
        const auto d_P_BASELINK_d_t_WORLD_BASELINK =
            -DPointRotationDPoint(R_BASELINK_WORLD, t_WORLD_BASELINK);
        Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>>
            d_E_d_t_WORLD_BASELINK(jacobians[1]);
        d_E_d_t_WORLD_BASELINK = d_E_d_P_CAMERA * d_P_CAMERA_d_P_BASELINK *
                                 d_P_BASELINK_d_t_WORLD_BASELINK;
        d_E_d_t_WORLD_BASELINK = -d_E_d_t_WORLD_BASELINK;
      }

      if (jacobians[2]) {
        Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> d_E_D_P_WORLD(
            jacobians[2]);
        // compute d(E)/d(P_WORLD) = d(E)/d(P_CAMERA) *
        // d(P_CAMERA)/d(P_BASELINK) * d(P_BASELINK)/d(P_WORLD)
        auto d_P_BASELINK_d_P_WORLD =
            DPointRotationDPoint(R_BASELINK_WORLD, P_WORLD);
        d_E_D_P_WORLD =
            d_E_d_P_CAMERA * d_P_CAMERA_d_P_BASELINK * d_P_BASELINK_d_P_WORLD;
        d_E_D_P_WORLD = -d_E_D_P_WORLD;
      }
    }
    return true;
  }

private:
  Eigen::Matrix2d A_; //!< The residual weighting matrix
  Eigen::Vector2d b_; //!< The measured pixel value
  Eigen::Matrix3d intrinsic_matrix_;
  Eigen::Matrix4d T_cam_baselink_;
};

} // namespace bs_constraints

#endif // FUSE_MODELS_EUCLIDEAN_REPROJECTION_FUNCTION_H
