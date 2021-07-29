#include <gtest/gtest.h>

#include <ctime>
#include <random>

#include <fuse_constraints/absolute_pose_3d_stamped_constraint.h>
#include <fuse_core/constraint.h>
#include <fuse_core/uuid.h>
#include <fuse_core/variable.h>
#include <fuse_graphs/hash_graph.h>
#include <pcl/io/pcd_io.h>

#include <beam_filtering/VoxelDownsample.h>
#include <beam_matching/Matchers.h>
#include <beam_utils/math.h>
#include <beam_utils/pointclouds.h>

#include <bs_common/sensor_proc.h>
#include <bs_common/utils.h>
#include <bs_common/scan_pose.h>

using namespace bs_common;

class Data {
 public:
  Data() {
    // read input cloud
    std::string current_file = "scan_pose_tests.cpp";
    std::string test_path = __FILE__;
    test_path.erase(test_path.end() - current_file.size(), test_path.end());
    std::string scan_path = test_path + "data/test_scan_vlp16.pcd";
    PointCloud test_cloud_tmp;
    PointCloud test_cloud;
    pcl::io::loadPCDFile(scan_path, test_cloud_tmp);

    // downsample input cloud
    Eigen::Vector3f scan_voxel_size(0.05, 0.05, 0.05);
    beam_filtering::VoxelDownsample downsampler(scan_voxel_size);
    downsampler.Filter(test_cloud_tmp, test_cloud);

    // create poses
    // srand(time(NULL));
    double max_pose_rot{20};
    double max_pose_trans{1};
    double max_pert_rot{10};
    double max_pert_trans{0.05};

    T_WORLD_S1 = Eigen::Matrix4d::Identity();
    Eigen::VectorXd perturb(6);
    perturb << beam::randf(max_pose_rot, -max_pose_rot),
        beam::randf(max_pose_rot, -max_pose_rot),
        beam::randf(max_pose_rot, -max_pose_rot),
        beam::randf(max_pose_trans, -max_pose_trans),
        beam::randf(max_pose_trans, -max_pose_trans),
        beam::randf(max_pose_trans, -max_pose_trans);
    T_WORLD_S2 = beam::PerturbTransformDegM(T_WORLD_S1, perturb);
    Eigen::VectorXd perturb2(6);
    perturb2 << beam::randf(max_pert_rot, -max_pert_rot),
        beam::randf(max_pert_rot, -max_pert_rot),
        beam::randf(max_pert_rot, -max_pert_rot),
        beam::randf(max_pert_trans, -max_pert_trans),
        beam::randf(max_pert_trans, -max_pert_trans),
        beam::randf(max_pert_trans, -max_pert_trans);
    T_WORLD_S2_pert = beam::PerturbTransformDegM(T_WORLD_S2, perturb2);
    T_S1_S2 = beam::InvertTransform(T_WORLD_S1) * T_WORLD_S2;

    Eigen::VectorXd perturb3(6);
    perturb3 << beam::randf(max_pose_rot, -max_pose_rot),
        beam::randf(max_pose_rot, -max_pose_rot),
        beam::randf(max_pose_rot, -max_pose_rot),
        beam::randf(max_pose_trans, -max_pose_trans),
        beam::randf(max_pose_trans, -max_pose_trans),
        beam::randf(max_pose_trans, -max_pose_trans);
    T_WORLD_S3 = beam::PerturbTransformDegM(T_WORLD_S2, perturb3);
    Eigen::VectorXd perturb4(6);
    perturb4 << beam::randf(max_pert_rot, -max_pert_rot),
        beam::randf(max_pert_rot, -max_pert_rot),
        beam::randf(max_pert_rot, -max_pert_rot),
        beam::randf(max_pert_trans, -max_pert_trans),
        beam::randf(max_pert_trans, -max_pert_trans),
        beam::randf(max_pert_trans, -max_pert_trans);
    T_WORLD_S3_pert = beam::PerturbTransformDegM(T_WORLD_S3, perturb4);
    T_S1_S3 = beam::InvertTransform(T_WORLD_S1) * T_WORLD_S3;
    T_S2_S3 = beam::InvertTransform(T_WORLD_S2) * T_WORLD_S3;

    // create scans
    S1 = test_cloud;
    pcl::transformPointCloud(S1, S2, beam::InvertTransform(T_S1_S2));
    pcl::transformPointCloud(S1, S3, beam::InvertTransform(T_S1_S3));

    // create scan matcher params
    matcher_params.max_corr = 1;
    matcher_params.max_iter = 50;
    matcher_params.t_eps = 1e-8;
    matcher_params.fit_eps = 1e-2;
    matcher_params.lidar_ang_covar = 7.78e-9;
    matcher_params.lidar_lin_covar = 2.5e-4;
    matcher_params.multiscale_steps = 0;
    matcher_params.res = 0;

    // create prior that will be used in optimization
    ScanPose SP_TMP(ros::Time(0), T_WORLD_S1, "fr", "fs", S1);

    fuse_core::Vector7d mean;
    mean << SP_TMP.Position().x(), SP_TMP.Position().y(), SP_TMP.Position().z(),
        SP_TMP.Orientation().w(), SP_TMP.Orientation().x(),
        SP_TMP.Orientation().y(), SP_TMP.Orientation().z();
    fuse_core::Matrix6d prior_covariance;
    prior_covariance.setIdentity();
    prior_covariance = prior_covariance * 0.0000000001;

    prior = std::make_shared<fuse_constraints::AbsolutePose3DStampedConstraint>(
        "PRIOR", SP_TMP.Position(), SP_TMP.Orientation(), mean,
        prior_covariance);
  }

  Eigen::Matrix4d T_WORLD_S1;
  Eigen::Matrix4d T_WORLD_S2;
  Eigen::Matrix4d T_WORLD_S2_pert;
  Eigen::Matrix4d T_WORLD_S3;
  Eigen::Matrix4d T_WORLD_S3_pert;
  Eigen::Matrix4d T_S1_S2;
  Eigen::Matrix4d T_S1_S3;
  Eigen::Matrix4d T_S2_S3;
  PointCloud S1;
  PointCloud S2;
  PointCloud S3;
  beam_matching::IcpMatcherParams matcher_params;
  fuse_constraints::AbsolutePose3DStampedConstraint::SharedPtr prior;
};

Data data_;

fuse_constraints::RelativePose3DStampedConstraint::SharedPtr CreateConstraint(
    const fuse_variables::Position3DStamped& position1,
    const fuse_variables::Orientation3DStamped& orientation1,
    const fuse_variables::Position3DStamped& position2,
    const fuse_variables::Orientation3DStamped& orientation2,
    const Eigen::Matrix4d& T_CLOUD1_CLOUD2) {
  // convert rotation matrix to quaternion
  Eigen::Matrix3d R = T_CLOUD1_CLOUD2.block(0, 0, 3, 3);
  Eigen::Quaterniond q(R);

  // Convert measurement to fuse variable
  fuse_core::Vector7d pose_relative_mean;
  pose_relative_mean << T_CLOUD1_CLOUD2(0, 3), T_CLOUD1_CLOUD2(1, 3),
      T_CLOUD1_CLOUD2(2, 3), q.w(), q.x(), q.y(), q.z();

  // create covariance
  Eigen::Matrix<double, 6, 6> covariance;
  covariance.setIdentity();
  covariance = covariance * 0.1;

  // Create a relative pose constraint. We assume the pose measurements are
  // independent.
  auto constraint =
      fuse_constraints::RelativePose3DStampedConstraint::make_shared(
          "SOURCE", position1, orientation1, position2, orientation2,
          pose_relative_mean, covariance);
  return constraint;
}

TEST(ScanPose, IO) {
  // create scan poses
  ScanPose SP1(ros::Time(0), data_.T_WORLD_S1, "fr", "fs", data_.S1);
  ScanPose SP2(ros::Time(1), data_.T_WORLD_S2, "fr", "fs", data_.S2);

  Eigen::Matrix4d T_WORLD_S2_ = SP2.T_REFFRAME_CLOUD();
  Eigen::Matrix4d T_WORLD_S1_ = SP1.T_REFFRAME_CLOUD();
  EXPECT_TRUE(beam::ArePosesEqual(data_.T_WORLD_S1, T_WORLD_S1_, 0.1, 0.001));
  EXPECT_EQ(SP1.Cloud().size(), data_.S1.size());
  EXPECT_EQ(SP2.Cloud().size(), data_.S2.size());
  EXPECT_EQ(SP1.Stamp(), ros::Time(0));
  EXPECT_EQ(SP2.Stamp(), ros::Time(1));
}

TEST(ScanPose, 2NodeFG) {
  // create scan poses
  ScanPose SP1(ros::Time(0), data_.T_WORLD_S1, "fr", "fs", data_.S1);
  ScanPose SP2(ros::Time(1), data_.T_WORLD_S2, "fr", "fs", data_.S2);

  // Create the graph
  fuse_graphs::HashGraph graph;

  // Add variables
  fuse_variables::Position3DStamped::SharedPtr p1 =
      fuse_variables::Position3DStamped::make_shared(SP1.Position());
  fuse_variables::Orientation3DStamped::SharedPtr o1 =
      fuse_variables::Orientation3DStamped::make_shared(SP1.Orientation());
  fuse_variables::Position3DStamped::SharedPtr p2 =
      fuse_variables::Position3DStamped::make_shared(SP2.Position());
  fuse_variables::Orientation3DStamped::SharedPtr o2 =
      fuse_variables::Orientation3DStamped::make_shared(SP2.Orientation());

  graph.addVariable(p1);
  graph.addVariable(o1);
  graph.addVariable(p2);
  graph.addVariable(o2);

  // Add constraint
  auto constraint1 = CreateConstraint(*p1, *o1, *p2, *o2, data_.T_S1_S2);
  graph.addConstraint(constraint1);

  // Optimize the constraints and variables.
  graph.optimize();

  for (int i = 0; i < 3; i++) {
    EXPECT_TRUE(p1->data()[i] == SP1.Position().data()[i]);
    EXPECT_TRUE(p2->data()[i] == SP2.Position().data()[i]);
  }
  for (int i = 0; i < 4; i++) {
    EXPECT_TRUE(o1->data()[i] == SP1.Orientation().data()[i]);
    EXPECT_TRUE(o2->data()[i] == SP2.Orientation().data()[i]);
  }
}

TEST(ScanPose, 2NodeFGPert) {
  // create scan poses
  ScanPose SP1(ros::Time(0), data_.T_WORLD_S1, "fr", "fs", data_.S1);
  ScanPose SP2(ros::Time(1), data_.T_WORLD_S2, "fr", "fs", data_.S2);
  ScanPose SP2_pert(ros::Time(1), data_.T_WORLD_S2_pert, "fr", "fs", data_.S2);

  // Create the graph
  fuse_graphs::HashGraph graph;

  // Add variables
  fuse_variables::Position3DStamped::SharedPtr p1 =
      fuse_variables::Position3DStamped::make_shared(SP1.Position());
  fuse_variables::Orientation3DStamped::SharedPtr o1 =
      fuse_variables::Orientation3DStamped::make_shared(SP1.Orientation());
  fuse_variables::Position3DStamped::SharedPtr p2 =
      fuse_variables::Position3DStamped::make_shared(SP2_pert.Position());
  fuse_variables::Orientation3DStamped::SharedPtr o2 =
      fuse_variables::Orientation3DStamped::make_shared(SP2_pert.Orientation());
  graph.addVariable(p1);
  graph.addVariable(o1);
  graph.addVariable(p2);
  graph.addVariable(o2);

  // Add constraint between poses
  auto constraint1 = CreateConstraint(*p1, *o1, *p2, *o2, data_.T_S1_S2);
  graph.addConstraint(constraint1);

  // Add prior on first pose
  graph.addConstraint(data_.prior);

  // Optimize the constraints and variables.
  graph.optimize();

  for (int i = 0; i < 3; i++) {
    EXPECT_TRUE(std::abs(SP1.Position().data()[i] - p1->data()[i]) < 0.001);
    EXPECT_TRUE(std::abs(SP2.Position().data()[i] - p2->data()[i]) < 0.001);
  }
  for (int i = 0; i < 4; i++) {
    EXPECT_TRUE(std::abs(SP1.Orientation().data()[i] - o1->data()[i]) < 0.001);
    EXPECT_TRUE(std::abs(SP2.Orientation().data()[i] - o2->data()[i]) < 0.001);
  }
}

TEST(ScanPose, ScanRegistrationICP) {
  // create scan poses
  ScanPose SP1(ros::Time(0), data_.T_WORLD_S1, "fr", "fs", data_.S1);
  ScanPose SP2(ros::Time(1), data_.T_WORLD_S2, "fr", "fs", data_.S2);
  ScanPose SP2_pert(ros::Time(1), data_.T_WORLD_S2_pert, "fr", "fs", data_.S2);

  Eigen::Matrix4d T_S1_S2_initial =
      beam::InvertTransform(data_.T_WORLD_S1) * data_.T_WORLD_S2_pert;
  PointCloudPtr S2_RefFEst = std::make_shared<PointCloud>();
  pcl::transformPointCloud(SP2_pert.Cloud(), *S2_RefFEst,
                           Eigen::Affine3d(T_S1_S2_initial));

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> matcher;
  matcher.setInputSource(S2_RefFEst);
  matcher.setInputTarget(std::make_shared<PointCloud>(SP1.Cloud()));
  matcher.setMaximumIterations(50);
  matcher.setTransformationEpsilon(1e-8);
  matcher.setMaxCorrespondenceDistance(1);
  matcher.setEuclideanFitnessEpsilon(1e-2);

  PointCloud S2_RefFOpt1;
  matcher.align(S2_RefFOpt1);
  Eigen::Matrix4d T_S1Opt_S1Ini =
      matcher.getFinalTransformation().cast<double>();
  Eigen::Matrix4d T_S1_S2_opt = T_S1Opt_S1Ini * T_S1_S2_initial;

  PointCloud S2_RefFOpt2;
  pcl::transformPointCloud(SP2_pert.Cloud(), S2_RefFOpt2, T_S1_S2_opt);

  // check final transform is close to original
  EXPECT_NEAR(data_.T_S1_S2.norm(), T_S1_S2_opt.norm(), 0.001);
}

TEST(ScanPose, ScanRegistrationPGPert) {
  // create scan poses
  ScanPose SP1(ros::Time(0), data_.T_WORLD_S1, "fr", "fs", data_.S1);
  ScanPose SP2(ros::Time(1), data_.T_WORLD_S2, "fr", "fs", data_.S2);
  ScanPose SP2_pert(ros::Time(1), data_.T_WORLD_S2_pert, "fr", "fs", data_.S2);

  // run scan registration
  // * (1) transform second scan into estimated scan 1 frame
  // * (2) run scan macher
  // * (3) get estimated transform between two frames
  //
  Eigen::Matrix4d T_S1_S2_init =
      beam::InvertTransform(data_.T_WORLD_S1) * data_.T_WORLD_S2_pert;
  PointCloudPtr S2_RefFEst = std::make_shared<PointCloud>();
  pcl::transformPointCloud(SP2_pert.Cloud(), *S2_RefFEst,
                           Eigen::Affine3d(T_S1_S2_init));

  std::unique_ptr<beam_matching::IcpMatcher> matcher =
      std::make_unique<beam_matching::IcpMatcher>(data_.matcher_params);
  matcher->SetRef(S2_RefFEst);
  matcher->SetTarget(std::make_shared<PointCloud>(SP1.Cloud()));
  matcher->Match();
  matcher->EstimateInfo();

  Eigen::Matrix4d T_S1Opt_S1Ini = matcher->GetResult().matrix();
  Eigen::Matrix4d T_S1_S2_opt = T_S1Opt_S1Ini * T_S1_S2_init;
  Eigen::Matrix<double, 6, 6> covariance = matcher->GetInfo();

  Eigen::Matrix4d T_W_S2_opt = data_.T_WORLD_S1 * T_S1_S2_opt;

  // check final transform is close to original
  EXPECT_NEAR(data_.T_S1_S2.norm(), T_S1_S2_opt.norm(), 0.001);
  EXPECT_NEAR(data_.T_WORLD_S2.norm(), T_W_S2_opt.norm(), 0.001);

  // Create the graph
  fuse_graphs::HashGraph graph;

  // Add variables
  fuse_variables::Position3DStamped::SharedPtr p1 =
      fuse_variables::Position3DStamped::make_shared(SP1.Position());
  fuse_variables::Orientation3DStamped::SharedPtr o1 =
      fuse_variables::Orientation3DStamped::make_shared(SP1.Orientation());
  fuse_variables::Position3DStamped::SharedPtr p2 =
      fuse_variables::Position3DStamped::make_shared(SP2_pert.Position());
  fuse_variables::Orientation3DStamped::SharedPtr o2 =
      fuse_variables::Orientation3DStamped::make_shared(SP2_pert.Orientation());

  graph.addVariable(p1);
  graph.addVariable(o1);
  graph.addVariable(p2);
  graph.addVariable(o2);

  // Add constraint between poses
  auto constraint1 = CreateConstraint(*p1, *o1, *p2, *o2, T_S1_S2_opt);
  graph.addConstraint(constraint1);

  // Add prior on first pose
  graph.addConstraint(data_.prior);

  // Optimize the constraints and variables.
  graph.optimize();

  for (int i = 0; i < 3; i++) {
    EXPECT_TRUE(std::abs(SP1.Position().data()[i] - p1->data()[i]) < 0.001);
    EXPECT_TRUE(std::abs(SP2.Position().data()[i] - p2->data()[i]) < 0.001);
  }
  for (int i = 0; i < 4; i++) {
    EXPECT_TRUE(std::abs(SP1.Orientation().data()[i] - o1->data()[i]) < 0.001);
    EXPECT_TRUE(std::abs(SP2.Orientation().data()[i] - o2->data()[i]) < 0.001);
  }
}
