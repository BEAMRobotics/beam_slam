#include <beam_utils/utils.h>
#include <gtest/gtest.h>

#include <bs_common/imu_state.h>
#include <bs_constraints/jacobians.h>

constexpr double EPS = 1e-6;
constexpr double THRESHOLD = 1e-6;
constexpr int N = 50;

TEST(EuclideanReprojectionFunction, Validity) {
  // create lambda for function to test
  auto projection = [&](const auto& q_WORLD_BASELINK,
                        const auto& t_WORLD_BASELINK, const auto& P_WORLD,
                        const auto& K, const auto& T_cam_baselink) {
    // get robot pose as a transformation matrix
    Eigen::Matrix4d T_WORLD_BASELINK = Eigen::Matrix4d::Identity();
    T_WORLD_BASELINK.block<3, 3>(0, 0) = q_WORLD_BASELINK.toRotationMatrix();
    T_WORLD_BASELINK.block<3, 1>(0, 3) = t_WORLD_BASELINK;

    // transform landmark into camera frame
    Eigen::Matrix4d T_CAMERA_WORLD =
        T_cam_baselink * beam::InvertTransform(T_WORLD_BASELINK);
    Eigen::Vector3d P(P_WORLD);
    Eigen::Vector3d P_CAMERA = (T_CAMERA_WORLD * P.homogeneous()).hnormalized();

    return (K * P_CAMERA).hnormalized()
  };

  // generate random P_WORLDcatkin build -
  // generate random q_WORLD_BASELINK
  // generate random t_WORLD_BASELINK
  // generate random K
  // generate random T_cam_baselink

  EuclideanReprojection reprojection_function(Eigen::Matrix2d::Identity(),
                                              pixel, K, T_cam_baselink);

  // form parameters pointer
  reprojection_function.Evaluate(parameters, residual, jacobians);

  // calculate numerical jacobian in blocks (quaternion, translation, point)
  Eigen::Matrix<double, 2, 10> J_numerical;


  EXPECT_TRUE(J_numerical.isApprox(*J_analytical, THRESHOLD));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
