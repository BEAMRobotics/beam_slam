#pragma once

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

class EuclideanReprojection : public ceres::SizedCostFunction<2, 4, 3, 3> {
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
  EuclideanReprojection(const Eigen::Matrix2d& information_matrix,
                        const Eigen::Vector2d& pixel_measurement,
                        const Eigen::Matrix3d& intrinsic_matrix,
                        const Eigen::Matrix4d& T_cam_baselink,
                        const Eigen::Matrix<double, 6, 6> pose_covariance =
                            Eigen::Matrix<double, 6, 6>::Identity())
      : information_matrix_(information_matrix),
        pixel_measurement_(pixel_measurement),
        intrinsic_matrix_(intrinsic_matrix),
        T_cam_baselink_(T_cam_baselink),
        pose_covariance_(pose_covariance) {}

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

    Eigen::Matrix3d R_CAM_BASELINK = T_cam_baselink_.block<3, 3>(0, 0);
    Eigen::Vector3d t_CAM_BASELINK = T_cam_baselink_.block<3, 1>(0, 3);
    Eigen::Matrix3d R_BASELINK_WORLD = R_WORLD_BASELINK.transpose();

    // 1. transform point into baselink frame
    Eigen::Vector3d P_BASELINK =
        (R_BASELINK_WORLD * P_WORLD) - (R_BASELINK_WORLD * t_WORLD_BASELINK);

    // 2. transform point into camera frame
    auto P_CAMERA = R_CAM_BASELINK * P_BASELINK + t_CAM_BASELINK;

    // 3. project into image space
    Eigen::Vector2d reprojection = (intrinsic_matrix_ * P_CAMERA).hnormalized();

    Eigen::Matrix2d info_matrix;
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
      double fx = intrinsic_matrix_(0, 0);
      double fy = intrinsic_matrix_(1, 1);
      double gx = P_WORLD[0];
      double gy = P_WORLD[1];
      double gz = P_WORLD[2];
      double gz2 = gz * gz;
      double gxyz = (gx * gy) / gz2;
      Eigen::Matrix<double, 2, 6, Eigen::RowMajor> J;
      J << fx / gz, 0, -fx * (gx / gz2), -fx * gxyz,
          fx * (1 + (gx * gx) / gz2), -fx * gy / gz, 0, fy / gz,
          -fy * (gy / gz2), -fy * (1 + (gy * gy) / gz2), fy * gxyz,
          fy * gx / gz;
      info_matrix = J * pose_covariance_ * J.transpose();
    } else {
      info_matrix = information_matrix_;
    }

    // compute weighted reprojection error
    Eigen::Vector2d E = info_matrix * (pixel_measurement_ - reprojection);
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

        // // ! code that should theoretically work but doesn't:
        // const auto d_P_BASELINK_D_R_WORLD_BASELINK =
        //     DPointRotationDRotation(R_BASELINK_WORLD, P_WORLD) *
        //         DInverseRotationDRotation(R_WORLD_BASELINK) -
        //     DPointRotationDRotation(R_BASELINK_WORLD, t_WORLD_BASELINK) *
        //         DInverseRotationDRotation(R_WORLD_BASELINK);
        // const auto d_E_d_R_WORLD_BASELINK = d_E_d_P_CAMERA *
        //                                     d_P_CAMERA_d_P_BASELINK *
        //                                     d_P_BASELINK_D_R_WORLD_BASELINK;
        // // lift to jacobian representation
        // Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>>
        //     d_E_d_q_WORLD_BASELINK(jacobians[0]);
        // d_E_d_q_WORLD_BASELINK =
        //     d_E_d_R_WORLD_BASELINK * MinusJacobian(q_WORLD_BASELINK);
        // d_E_d_q_WORLD_BASELINK = -d_E_d_q_WORLD_BASELINK;

        // ! workaround for getting jacobian wrt quaternion:
        Eigen::Vector4d q_coeffs(q_WORLD_BASELINK.w(), q_WORLD_BASELINK.x(),
                                 q_WORLD_BASELINK.y(), q_WORLD_BASELINK.z());
        Eigen::Matrix<double, 3, 4> d_P_BASELINK_D_q_WORLD_BASELINK;
        const auto res =
            point_transformation(q_coeffs, t_WORLD_BASELINK, P_WORLD);
        for (int i = 0; i < 4; i++) {
          Eigen::Vector4d pert = Eigen::Vector4d::Zero();
          pert[i] = EPSILON;
          const auto res_pert =
              point_transformation(q_coeffs + pert, t_WORLD_BASELINK, P_WORLD);
          const auto finite_diff = (res_pert - res) / EPSILON;
          d_P_BASELINK_D_q_WORLD_BASELINK.col(i) = finite_diff.transpose();
        }

        Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>>
            d_E_d_q_WORLD_BASELINK(jacobians[0]);
        d_E_d_q_WORLD_BASELINK = d_E_d_P_CAMERA * d_P_CAMERA_d_P_BASELINK *
                                 d_P_BASELINK_D_q_WORLD_BASELINK;
        d_E_d_q_WORLD_BASELINK.applyOnTheLeft(-info_matrix);
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
        d_E_d_t_WORLD_BASELINK.applyOnTheLeft(-info_matrix);
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
        d_E_D_P_WORLD.applyOnTheLeft(-info_matrix);
      }
    }
    return true;
  }

private:
  Eigen::Matrix2d information_matrix_; //!< The residual weighting matrix
  Eigen::Vector2d pixel_measurement_;  //!< The measured pixel value
  Eigen::Matrix3d intrinsic_matrix_;
  Eigen::Matrix4d T_cam_baselink_;
  Eigen::Matrix<double, 6, 6> pose_covariance_;
};

} // namespace bs_constraints
