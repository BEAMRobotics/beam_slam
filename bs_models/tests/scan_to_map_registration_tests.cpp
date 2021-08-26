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

#include <bs_common/utils.h>
#include <bs_models/frame_to_frame/scan_registration/scan_to_map_registration.h>

using namespace bs_models;
using namespace frame_to_frame;
using namespace bs_common;

class Data {
 public:
  Data() {
    // read input cloud
    std::string current_file = "scan_to_map_registration_tests.cpp";
    std::string test_path = __FILE__;
    test_path.erase(test_path.end() - current_file.size(), test_path.end());
    std::string scan_path = test_path + "data/test_scan_vlp16.pcd";
    PointCloud test_cloud_tmp;
    PointCloud test_cloud;
    pcl::io::loadPCDFile(scan_path, test_cloud_tmp);

    // create loam params from config
    std::string config_path = test_path + "data/loam_config.json";
    loam_params = std::make_shared<LoamParams>(config_path);

    // create scan reg params
    scan_reg_params.outlier_threshold_trans_m = 1;
    scan_reg_params.outlier_threshold_rot_deg = 30;
    scan_reg_params.min_motion_trans_m = 0;
    scan_reg_params.min_motion_rot_deg = 0;
    scan_reg_params.fix_first_scan = true;
    scan_reg_params.map_size = 3;
    scan_reg_params.store_full_cloud = false;

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
  }

  Eigen::Matrix4d T_WORLD_S1;
  Eigen::Matrix4d T_WORLD_S2;
  Eigen::Matrix4d T_WORLD_S2_pert;
  Eigen::Matrix4d T_WORLD_S3;
  Eigen::Matrix4d T_WORLD_S3_pert;
  Eigen::Matrix4d T_S1_S2;
  Eigen::Matrix4d T_S1_S3;
  Eigen::Matrix4d T_S2_S3;
  Eigen::Matrix4d T_BASELINK_LIDAR = Eigen::Matrix4d::Identity();
  PointCloud S1;
  PointCloud S2;
  PointCloud S3;
  std::shared_ptr<LoamParams> loam_params;
  ScanToMapLoamRegistration::Params scan_reg_params;
};

Data data_;

TEST(ScanToMapLoamRegistration, 2Scans) {
  data_ = Data();

  // init scan registration
  std::unique_ptr<Matcher<LoamPointCloudPtr>> matcher;
  matcher = std::make_unique<LoamMatcher>(*data_.loam_params);
  std::shared_ptr<LoamFeatureExtractor> feature_extractor =
      std::make_shared<LoamFeatureExtractor>(data_.loam_params);

  // create scan poses
  ScanPose SP1(data_.S1, ros::Time(0), data_.T_WORLD_S1, data_.T_BASELINK_LIDAR,
               feature_extractor);
  ScanPose SP2(data_.S2, ros::Time(1), data_.T_WORLD_S2, data_.T_BASELINK_LIDAR,
               feature_extractor);
  ScanPose SP2_pert(data_.S2, ros::Time(1), data_.T_WORLD_S2_pert,
                    data_.T_BASELINK_LIDAR, feature_extractor);

  std::unique_ptr<ScanToMapLoamRegistration> scan_registration =
      std::make_unique<ScanToMapLoamRegistration>(
          std::move(matcher), data_.scan_reg_params.GetBaseParams(),
          data_.scan_reg_params.map_size,
          data_.scan_reg_params.store_full_cloud);

  Eigen::Matrix<double, 6, 6> covariance;
  covariance.setIdentity();
  covariance = covariance * 0.1;
  scan_registration->SetFixedCovariance(covariance);

  auto transaction1 = scan_registration->RegisterNewScan(SP1).GetTransaction();
  auto transaction2 =
      scan_registration->RegisterNewScan(SP2_pert).GetTransaction();

  // Create the graph
  fuse_graphs::HashGraph graph;

  // add transactions
  graph.update(*transaction1);
  graph.update(*transaction2);

  // Optimize the constraints and variables.
  graph.optimize();

  auto p1 = dynamic_cast<const fuse_variables::Position3DStamped&>(
      graph.getVariable(SP1.Position().uuid()));
  auto p2 = dynamic_cast<const fuse_variables::Position3DStamped&>(
      graph.getVariable(SP2.Position().uuid()));

  auto o1 = dynamic_cast<const fuse_variables::Orientation3DStamped&>(
      graph.getVariable(SP1.Orientation().uuid()));
  auto o2 = dynamic_cast<const fuse_variables::Orientation3DStamped&>(
      graph.getVariable(SP2.Orientation().uuid()));

  Eigen::Matrix4d T_WORLD_S1_mea = bs_common::FusePoseToEigenTransform(p1, o1);
  Eigen::Matrix4d T_WORLD_S2_mea = bs_common::FusePoseToEigenTransform(p2, o2);

  EXPECT_TRUE(
      beam::ArePosesEqual(T_WORLD_S1_mea, data_.T_WORLD_S1, 1, 0.005, true));
  EXPECT_TRUE(
      beam::ArePosesEqual(T_WORLD_S2_mea, data_.T_WORLD_S2, 1, 0.06, true));
}

TEST(ScanToMapLoamRegistration, 3Scans) {
  data_ = Data();

  // init scan registration
  std::unique_ptr<Matcher<LoamPointCloudPtr>> matcher;
  matcher = std::make_unique<LoamMatcher>(*data_.loam_params);
  std::shared_ptr<LoamFeatureExtractor> feature_extractor =
      std::make_shared<LoamFeatureExtractor>(data_.loam_params);

  // create scan poses
  ScanPose SP1(data_.S1, ros::Time(0), data_.T_WORLD_S1, data_.T_BASELINK_LIDAR,
               feature_extractor);
  ScanPose SP2(data_.S2, ros::Time(1), data_.T_WORLD_S2, data_.T_BASELINK_LIDAR,
               feature_extractor);
  ScanPose SP3(data_.S3, ros::Time(2), data_.T_WORLD_S3, data_.T_BASELINK_LIDAR,
               feature_extractor);
  ScanPose SP2_pert(data_.S2, ros::Time(1), data_.T_WORLD_S2_pert,
                    data_.T_BASELINK_LIDAR, feature_extractor);
  ScanPose SP3_pert(data_.S3, ros::Time(2), data_.T_WORLD_S3_pert,
                    data_.T_BASELINK_LIDAR, feature_extractor);
  //   SP1.Save("/home/nick/tmp/loam_scan_registration/sp1/");
  //   SP2.Save("/home/nick/tmp/loam_scan_registration/sp2/");
  //   SP3.Save("/home/nick/tmp/loam_scan_registration/sp3/");

  // init scan registration
  std::unique_ptr<ScanToMapLoamRegistration> scan_registration =
      std::make_unique<ScanToMapLoamRegistration>(
          std::move(matcher), data_.scan_reg_params.GetBaseParams(),
          data_.scan_reg_params.map_size,
          data_.scan_reg_params.store_full_cloud);

  Eigen::Matrix<double, 6, 6> covariance;
  covariance.setIdentity();
  covariance = covariance * 0.1;
  scan_registration->SetFixedCovariance(covariance);

  // Create the graph
  fuse_graphs::HashGraph graph;

  // add transactions
  auto transaction1 = scan_registration->RegisterNewScan(SP1).GetTransaction();

  //   scan_registration->GetMap().Save("/home/nick/tmp/loam_scan_registration/maps/1/");
  EXPECT_EQ(scan_registration->GetMap().NumLoamClouds(), 1);
  EXPECT_EQ(scan_registration->GetMap().NumPointClouds(), 0);

  auto transaction2 =
      scan_registration->RegisterNewScan(SP2_pert).GetTransaction();

  //   scan_registration->GetMap().Save("/home/nick/tmp/loam_scan_registration/maps/2/");
  EXPECT_EQ(scan_registration->GetMap().NumLoamClouds(), 2);
  EXPECT_EQ(scan_registration->GetMap().NumPointClouds(), 0);

  auto transaction3 =
      scan_registration->RegisterNewScan(SP3_pert).GetTransaction();

  //   scan_registration->GetMap().Save("/home/nick/tmp/loam_scan_registration/maps/3/");
  EXPECT_EQ(scan_registration->GetMap().NumLoamClouds(), 3);
  EXPECT_EQ(scan_registration->GetMap().NumPointClouds(), 0);

  graph.update(*transaction1);
  graph.update(*transaction2);
  graph.update(*transaction3);
  graph.optimize();

  // Check results
  auto p1 = dynamic_cast<const fuse_variables::Position3DStamped&>(
      graph.getVariable(SP1.Position().uuid()));
  auto p2 = dynamic_cast<const fuse_variables::Position3DStamped&>(
      graph.getVariable(SP2.Position().uuid()));
  auto p3 = dynamic_cast<const fuse_variables::Position3DStamped&>(
      graph.getVariable(SP3.Position().uuid()));

  auto o1 = dynamic_cast<const fuse_variables::Orientation3DStamped&>(
      graph.getVariable(SP1.Orientation().uuid()));
  auto o2 = dynamic_cast<const fuse_variables::Orientation3DStamped&>(
      graph.getVariable(SP2.Orientation().uuid()));
  auto o3 = dynamic_cast<const fuse_variables::Orientation3DStamped&>(
      graph.getVariable(SP3.Orientation().uuid()));

  Eigen::Matrix4d T_WORLD_S1_mea = bs_common::FusePoseToEigenTransform(p1, o1);
  Eigen::Matrix4d T_WORLD_S2_mea = bs_common::FusePoseToEigenTransform(p2, o2);
  Eigen::Matrix4d T_WORLD_S3_mea = bs_common::FusePoseToEigenTransform(p3, o3);

  //   std::cout << "S1 ground truth: \n";
  //   beam::OutputTransformInformation(data_.T_WORLD_S1);
  //   std::cout << "S1 Measured: \n";
  //   beam::OutputTransformInformation(T_WORLD_S1_mea);

  //   std::cout << "\nS2 ground truth: \n";
  //   beam::OutputTransformInformation(data_.T_WORLD_S2);
  //   std::cout << "S2 Measured: \n";
  //   beam::OutputTransformInformation(T_WORLD_S2_mea);

  //   std::cout << "\nS3 ground truth: \n";
  //   beam::OutputTransformInformation(data_.T_WORLD_S3);
  //   std::cout << "S3 Measured: \n";
  //   beam::OutputTransformInformation(T_WORLD_S3_mea);

  EXPECT_TRUE(
      beam::ArePosesEqual(T_WORLD_S1_mea, data_.T_WORLD_S1, 1, 0.005, true));
  EXPECT_TRUE(
      beam::ArePosesEqual(T_WORLD_S2_mea, data_.T_WORLD_S2, 1, 0.06, true));
  EXPECT_TRUE(
      beam::ArePosesEqual(T_WORLD_S3_mea, data_.T_WORLD_S3, 1, 0.06, true));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
