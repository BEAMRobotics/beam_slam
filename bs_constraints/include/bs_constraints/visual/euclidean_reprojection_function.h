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
    Eigen::Quaterniond q_WORLD_BASELINK(parameters[0][3], parameters[0][0],
                                        parameters[0][1], parameters[0][2]);
    Eigen::Vector3d t_WORLD_BASELINK(parameters[1][0], parameters[1][1],
                                     parameters[1][2]);
    Eigen::Vector3d P_WORLD(parameters[2][0], parameters[2][1],
                            parameters[2][2]);

    // get robot pose as a transformation matrix
    Eigen::Matrix4d T_WORLD_BASELINK = Eigen::Matrix4d::Identity();
    T_WORLD_BASELINK.block<3, 3>(0, 0) = q_WORLD_BASELINK.toRotationMatrix();
    T_WORLD_BASELINK.block<3, 1>(0, 3) = t_WORLD_BASELINK;

    Eigen::Matrix4d T_BASELINK_WORLD = beam::InvertTransform(T_WORLD_BASELINK);

    // transform landmark into camera frame
    Eigen::Matrix4d T_CAMERA_WORLD = T_cam_baselink_ * T_BASELINK_WORLD;
    Eigen::Vector3d P_CAMERA =
        (T_CAMERA_WORLD * P_WORLD.homogeneous()).hnormalized();

    // project into image space
    Eigen::Vector2d reprojection = (intrinsic_matrix_ * P_CAMERA).hnormalized();

    // compute weighted reprojection error
    Eigen::Vector2d E = A_ * (b_ - reprojection);
    residual[0] = E[0];
    residual[1] = E[1];

    // compute jacobians
    if (jacobians) {
      const auto d_E_d_P_CAMERA =
          DImageProjectionDPoint(intrinsic_matrix_, P_CAMERA);

      const auto d_P_CAMERA_d_T_CAMERA_WORLD =
          DPointTransformationDTransform(T_CAMERA_WORLD, P_WORLD);

      if (jacobians[0] || jacobians[1]) {
        // compute d(E)/d(T_WORLD_BASELINK) = d(E)/d(P_CAMERA) *
        // d(P_CAMERA)/d(T_CAMERA_WORLD) *
        // d(T_CAMERA_WORLD)/d(T_BASELINK_WORLD) *
        // d(T_BASELINK_WORLD)/d(T_WORLD_BASELINK)

        const auto d_T_CAMERA_WORLD_d_T_BASELINK_WORLD =
            DTransformCompositionDRightTransform(T_cam_baselink_,
                                                 T_BASELINK_WORLD);

        const auto d_T_BASELINK_WORLD_d_T_WORLD_BASELINK =
            DInverseTransformDTransform(T_WORLD_BASELINK);

        // jacobian for full transform (2x6 : qx qy qz x y z)
        auto d_E_d_T_WORLD_BASELINK = d_E_d_P_CAMERA *
                                      d_P_CAMERA_d_T_CAMERA_WORLD *
                                      d_T_CAMERA_WORLD_d_T_BASELINK_WORLD *
                                      d_T_BASELINK_WORLD_d_T_WORLD_BASELINK;

        if (jacobians[0]) {
          // far right column = 0 due to compact quaternion representation
          Eigen::Matrix<double, 2, 4> d_E_d_R_WORLD_BASELINK =
              Eigen::Matrix<double, 2, 4>::Zero();
          d_E_d_R_WORLD_BASELINK.block<2, 3>(0, 0) =
              d_E_d_T_WORLD_BASELINK.block<2, 3>(0, 0);

          Eigen::Map<Eigen::MatrixXd>(
              jacobians[0], d_E_d_R_WORLD_BASELINK.rows(),
              d_E_d_R_WORLD_BASELINK.cols()) = d_E_d_R_WORLD_BASELINK;
        }
        if (jacobians[1]) {
          Eigen::Matrix<double, 2, 3> d_E_d_t_WORLD_BASELINK =
              d_E_d_T_WORLD_BASELINK.block<2, 3>(0, 3);
          Eigen::Map<Eigen::MatrixXd>(
              jacobians[1], d_E_d_t_WORLD_BASELINK.rows(),
              d_E_d_t_WORLD_BASELINK.cols()) = d_E_d_t_WORLD_BASELINK;
        }
      }

      if (jacobians[2]) {
        // compute d(E)/d(P_WORLD) = d(E)/d(P_CAMERA) * d(P_CAMERA)/d(P_WORLD)
        auto d_P_CAMERA_D_P_WORLD = DPointTransformationDPoint(T_CAMERA_WORLD);
        auto d_E_D_P_WORLD = d_E_d_P_CAMERA * d_P_CAMERA_D_P_WORLD;

        Eigen::Map<Eigen::MatrixXd>(jacobians[2], d_E_D_P_WORLD.rows(),
                                    d_E_D_P_WORLD.cols()) = d_E_D_P_WORLD;
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
