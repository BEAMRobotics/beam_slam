#include <beam_utils/utils.h>
#include <gtest/gtest.h>

#include <bs_common/imu_state.h>
#include <bs_constraints/jacobians.h>
#include <bs_constraints/visual/euclidean_reprojection_function.h>
#include <bs_constraints/visual/reprojection_functor.h>

constexpr double EPS = 1e-8;
constexpr double THRESHOLD = 1e-6;
constexpr int N = 50;

Eigen::Matrix3d GenerateRandomIntrinsicMatrix() {
  Eigen::Vector2d camera_center = beam::UniformRandomVector<2>(100.0, 1000.0);
  double f = beam::randf(10.0, 100.0);
  Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
  K(0, 0) = f;
  K(1, 1) = f;
  K(0, 2) = camera_center.x();
  K(1, 2) = camera_center.y();
  return K;
}

TEST(EuclideanReprojectionFunction, Validity) {
  // create lambda for function to test
  auto projection = [&](const auto& params, const auto& K,
                        const auto& T_CAM_BASELINK) {
    Eigen::Quaterniond q_WORLD_BASELINK(params[0], params[1], params[2],
                                        params[3]);
    q_WORLD_BASELINK.normalize();
    Eigen::Vector3d t_WORLD_BASELINK(params[4], params[5], params[6]);
    Eigen::Vector3d P_WORLD(params[7], params[8], params[9]);

    Eigen::Matrix4d T_WORLD_BASELINK = Eigen::Matrix4d::Identity();
    T_WORLD_BASELINK.block<3, 3>(0, 0) = q_WORLD_BASELINK.toRotationMatrix();
    T_WORLD_BASELINK.block<3, 1>(0, 3) = t_WORLD_BASELINK;

    // transform landmark into camera frame
    Eigen::Matrix4d T_CAMERA_WORLD =
        T_CAM_BASELINK * beam::InvertTransform(T_WORLD_BASELINK);
    Eigen::Vector3d P_CAMERA =
        (T_CAMERA_WORLD * P_WORLD.homogeneous()).hnormalized();
    const Eigen::Vector2d pixel = (K * P_CAMERA).hnormalized();
    return pixel;
  };

  for (int i = 0; i < N; i++) {
    std::cout << "\nTest # " << i << std::endl;
    // generate random T_WORLD_BASELINK
    const Eigen::Matrix4d T_WORLD_BASELINK =
        beam::GenerateRandomPose(1.0, 10.0);
    const Eigen::Quaterniond q_WORLD_BASELINK(
        T_WORLD_BASELINK.block<3, 3>(0, 0));
    const Eigen::Vector3d t_WORLD_BASELINK =
        T_WORLD_BASELINK.block<3, 1>(0, 3).transpose();

    // generate random K
    Eigen::Matrix3d K = GenerateRandomIntrinsicMatrix();

    // generate random T_CAM_BASELINK
    const Eigen::Matrix4d T_CAM_BASELINK = beam::GenerateRandomPose(0.0, 1.0);

    // generate random P_WORLD
    const Eigen::Vector3d P_CAM =
        beam::randf(5.0, 10.0) *
        beam::UniformRandomVector<3>(0.1, 1.0).normalized();
    const Eigen::Vector3d P_WORLD =
        (T_WORLD_BASELINK * beam::InvertTransform(T_CAM_BASELINK) *
         P_CAM.homogeneous())
            .hnormalized();

    Eigen::Matrix<double, 10, 1> params;
    params << q_WORLD_BASELINK.w(), q_WORLD_BASELINK.x(), q_WORLD_BASELINK.y(),
        q_WORLD_BASELINK.z(), t_WORLD_BASELINK.x(), t_WORLD_BASELINK.y(),
        t_WORLD_BASELINK.z(), P_WORLD.x(), P_WORLD.y(), P_WORLD.z();

    // create raw parameters pointer
    double q_params[4] = {q_WORLD_BASELINK.w(), q_WORLD_BASELINK.x(),
                          q_WORLD_BASELINK.y(), q_WORLD_BASELINK.z()};
    double t_params[3] = {t_WORLD_BASELINK.x(), t_WORLD_BASELINK.y(),
                          t_WORLD_BASELINK.z()};
    double p_params[3] = {P_WORLD.x(), P_WORLD.y(), P_WORLD.z()};
    double* parameters[3] = {q_params, t_params, p_params};

    // compute its pixel projection
    const auto pixel = projection(params, K, T_CAM_BASELINK);

    Eigen::Matrix<double, 2, 10> J_numerical;
    Eigen::Matrix<double, 2, 10> J_analytical;
    Eigen::Matrix<double, 2, 10> J_auto;

    { // calculate analytical jacobian
      bs_constraints::EuclideanReprojection reprojection_function(
          Eigen::Matrix2d::Identity(), pixel, K, T_CAM_BASELINK);

      // evaulate reprojection and compute jacobians
      double residual[2];
      double* J[3];
      double J_q[8];
      double J_t[6];
      double J_p[6];
      J[0] = J_q;
      J[1] = J_t;
      J[2] = J_p;
      reprojection_function.Evaluate(parameters, residual, J);

      // clang-format off
      J_analytical << J[0][0], J[0][1], J[0][2], J[0][3], J[1][0], J[1][1], J[1][2], J[2][0], J[2][1], J[2][2],
                      J[0][4], J[0][5], J[0][6], J[0][7], J[1][3], J[1][4], J[1][5], J[2][3], J[2][4], J[2][5];

      // clang-format on
      std::cout << "Analytical jacobian: " << std::endl;
      std::cout << J_analytical << std::endl;
    }

    // { // calculate numerical jacobian
    //   for (int i = 0; i < 10; i++) {
    //     Eigen::Matrix<double, 10, 1> pert =
    //         Eigen::Matrix<double, 10, 1>::Zero();
    //     pert[i] = EPS;
    //     const auto pixel_pert = projection(params + pert, K, T_CAM_BASELINK);
    //     const auto finite_diff = (pixel - pixel_pert) / EPS;
    //     J_numerical.col(i) = finite_diff.transpose();
    //   }
    //   std::cout << "Numerical jacobian: " << std::endl;
    //   std::cout << J_numerical << std::endl;
    // }

    { // calculate autodiff jacobian
      ceres::CostFunction* cost =
          new ceres::AutoDiffCostFunction<bs_constraints::ReprojectionFunctor,
                                          2, 4, 3, 3>(
              new bs_constraints::ReprojectionFunctor(
                  Eigen::Matrix2d::Identity(), pixel, K, T_CAM_BASELINK));

      double residual[2];
      double* J[3];
      double J_q[8];
      double J_t[6];
      double J_p[6];
      J[0] = J_q;
      J[1] = J_t;
      J[2] = J_p;
      cost->Evaluate(parameters, residual, J);

      // clang-format off
      J_auto  << J[0][0], J[0][1], J[0][2], J[0][3], J[1][0], J[1][1], J[1][2], J[2][0], J[2][1], J[2][2],
                 J[0][4], J[0][5], J[0][6], J[0][7], J[1][3], J[1][4], J[1][5], J[2][3], J[2][4], J[2][5];
      // clang-format on
      std::cout << "Autodiff jacobian: " << std::endl;
      std::cout << J_auto << std::endl;
    }

    // EXPECT_TRUE(J_numerical.isApprox(J_analytical, THRESHOLD));
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
