#include <fuse_core/constraint.h>
#include <fuse_core/uuid.h>
#include <fuse_core/variable.h>
#include <fuse_graphs/hash_graph.h>
#include <gtest/gtest.h>

#include <beam_calibration/CameraModels.h>
#include <beam_cv/geometry/Triangulation.h>
#include <beam_utils/math.h>
#include <beam_utils/se3.h>
#include <bs_constraints/visual/reprojection_functor.h>

TEST(ReprojectionFunctor, TestAccuracy) {
  std::string current_file = "reprojection_test.cpp";
  std::string test_path = __FILE__;
  test_path.erase(test_path.end() - current_file.size(), test_path.end());
  std::string cam_loc = test_path + "data/intrinsics.json";
  std::shared_ptr<beam_calibration::CameraModel> cam =
      beam_calibration::CameraModel::Create(cam_loc);

  Eigen::Matrix4d T_imu_cam;
  T_imu_cam << 0.0148655429818, -0.999880929698, 0.00414029679422,
      -0.0216401454975, 0.999557249008, 0.0149672133247, 0.025715529948,
      -0.064676986768, -0.0257744366974, 0.00375618835797, 0.999660727178,
      0.00981073058949, 0.0, 0.0, 0.0, 1.0;

  // make a test camera pose
  Eigen::Matrix4d T_world_cam = Eigen::Matrix4d::Identity();
  Eigen::Vector3d position;
  position << 5, 5, 5;
  T_world_cam.block<3, 1>(0, 3) = position.transpose();
  Eigen::Matrix4d T_world_imu = T_world_cam * T_imu_cam.inverse();
  Eigen::Vector3d p;
  Eigen::Quaterniond q;
  beam::TransformMatrixToQuaternionAndTranslation(T_world_imu, q, p);

  Eigen::Vector2i pixel(cam->GetHeight() / 2, cam->GetWidth() / 2);
  Eigen::Vector3d point_cam;
  cam->BackProject(pixel, point_cam);
  point_cam = 10 * point_cam;
  Eigen::Vector4d point_cam_h;
  point_cam_h << point_cam[0], point_cam[1], point_cam[2], 1;
  Eigen::Vector3d point_world = (T_world_cam * point_cam_h).hnormalized();

  Eigen::Matrix2d A = Eigen::Matrix2d::Identity();
  Eigen::Vector2d pixeld = pixel.cast<double>();
  bs_constraints::ReprojectionFunctor reproj =
      bs_constraints::ReprojectionFunctor(A, pixeld, cam->GetIntrinsicMatrix(),
                                          T_imu_cam.inverse());

  double t_WORLD_IMU[3];
  t_WORLD_IMU[0] = p[0];
  t_WORLD_IMU[1] = p[1];
  t_WORLD_IMU[2] = p[2];

  double R_WORLD_IMU[4];
  R_WORLD_IMU[0] = q.w();
  R_WORLD_IMU[1] = q.x();
  R_WORLD_IMU[2] = q.y();
  R_WORLD_IMU[3] = q.z();

  double p_WORLD[3];
  p_WORLD[0] = point_world[0];
  p_WORLD[1] = point_world[1];
  p_WORLD[2] = point_world[2];

  double residual[2];
  residual[0] = 10;
  residual[1] = 10;
  reproj(R_WORLD_IMU, t_WORLD_IMU, p_WORLD, residual);

  EXPECT_NEAR(residual[0], 0, 0.001);
  EXPECT_NEAR(residual[1], 0, 0.001);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
