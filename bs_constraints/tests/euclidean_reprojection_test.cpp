#include <beam_utils/utils.h>
#include <gtest/gtest.h>

#include <bs_common/imu_state.h>
#include <bs_constraints/jacobians.h>
#include <bs_constraints/visual/euclidean_reprojection_function.h>

constexpr double EPS = 1e-8;
constexpr double THRESHOLD = 1e-6;
constexpr int N = 50;

TEST(EuclideanReprojectionFunction, Validity) {
  // create lambda for function to test
  auto projection = [&](const auto& T_WORLD_BASELINK, const auto& P_WORLD,
                        const auto& K, const auto& T_CAM_BASELINK) {
    // transform landmark into camera frame
    Eigen::Matrix4d T_CAMERA_WORLD =
        T_CAM_BASELINK * beam::InvertTransform(T_WORLD_BASELINK);
    Eigen::Vector3d P(P_WORLD);
    Eigen::Vector3d P_CAMERA = (T_CAMERA_WORLD * P.homogeneous()).hnormalized();

    return (K * P_CAMERA).hnormalized();
  };

  // generate random T_WORLD_BASELINK
  const Eigen::Matrix4d T_WORLD_BASELINK = beam::GenerateRandomPose(1.0, 10.0);
  const Eigen::Quaterniond q_WORLD_BASELINK(T_WORLD_BASELINK.block<3, 3>(0, 0));
  const Eigen::Vector3d t_WORLD_BASELINK =
      T_WORLD_BASELINK.block<3, 1>(0, 3).transpose();

  // generate random K
  Eigen::Vector2d camera_center = beam::UniformRandomVector<2>(50.0, 100.0);
  double f = beam::randf(10.0, 100.0);
  Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
  K(0, 0) = f;
  K(1, 1) = f;
  K(0, 2) = camera_center.x();
  K(1, 2) = camera_center.y();

  // generate random T_CAM_BASELINK
  const Eigen::Matrix4d T_CAM_BASELINK = beam::GenerateRandomPose(1.0, 10.0);

  // generate random P_WORLD
  const Eigen::Vector3d P_CAM =
      beam::randf(5.0, 10.0) *
      beam::UniformRandomVector<3>(0.1, 1.0).normalized();
  const Eigen::Vector3d P_WORLD =
      T_WORLD_BASELINK * beam::InvertTransform(T_CAM_BASELINK) * P_CAM;

  // compute its pixel projection
  const Eigen::Vector2d pixel =
      projection(T_WORLD_BASELINK, P_WORLD, K, T_CAM_BASELINK);

  EuclideanReprojection reprojection_function(Eigen::Matrix2d::Identity(),
                                              pixel, K, T_cam_baselink);

  // create raw parameters pointer
  double q_params[4];
  q_params[0] = q_WORLD_BASELINK.x();
  q_params[1] = q_WORLD_BASELINK.y();
  q_params[2] = q_WORLD_BASELINK.z();
  q_params[3] = q_WORLD_BASELINK.w();
  double t_params[3];
  t_params[0] = t_WORLD_BASELINK.x();
  t_params[1] = t_WORLD_BASELINK.y();
  t_params[2] = t_WORLD_BASELINK.z();
  double p_params[3];
  p_params[0] = P_WORLD.x();
  p_params[1] = P_WORLD.y();
  p_params[2] = P_WORLD.z();

  double** parameters[3];
  parameters[0] = q_params;
  parameters[1] = t_params;
  parameters[2] = p_params;

  // evaulate reprojection and compute jacobians
  double residual[2];
  double jacobians[20];
  reprojection_function.Evaluate(parameters, residual, jacobians);

  // calculate numerical jacobian
  Eigen::Matrix<double, 2, 10> J_numerical =
      Eigen::Matrix<double, 2, 10>::Zero();
  for (int i = 0; i < 7; i++) {
    Eigen::Matrix<double, 6, 1> pert = Eigen::Matrix<double, 6, 1>::Zero();
    if (i < 3) {
      pert[i] = EPS;
    } else if (i > 3) {
      pert[i - 1] = EPS;
    } else {
      // useless 4th parameter in quaternion
      continue;
    }
    const auto proj = projection(T_WORLD_BASELINK, P_WORLD, K, T_CAM_BASELINK);
    const auto proj_pert = projection(beam::BoxPlus(T_WORLD_BASELINK, pert),
                                      P_WORLD, K, T_CAM_BASELINK);
    const auto finite_diff = (proj_pert - proj) / EPS;
    J_numerical.col(i) = finite_diff.transpose();
  }
  for (int i = 7; i < 10; i++) {
    Eigen::Vector3d pert(0, 0, 0);
    pert[i] = EPS;
    const auto proj = projection(T_WORLD_BASELINK, P_WORLD, K, T_CAM_BASELINK);
    const auto proj_pert =
        projection(T_WORLD_BASELINK, P_WORLD + pert, K, T_CAM_BASELINK);
    const auto finite_diff = (proj_pert - proj) / EPS;
    J_numerical.col(i) = finite_diff.transpose();
  }

  Eigen::Matrix<double, 2, 10> J_analytical =
      Eigen::Map<Eigen::Matrix<double, 2, 10, RowMajor> >(jacobians);
  EXPECT_TRUE(J_numerical.isApprox(J_analytical, THRESHOLD));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
