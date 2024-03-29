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
#include <beam_utils/se3.h>
#include <beam_utils/simple_path_generator.h>

#include <bs_common/conversions.h>
#include <bs_common/utils.h>
#include <bs_models/scan_registration/multi_scan_registration.h>

#include <test_utils.h>

using namespace bs_models;
using namespace scan_registration;
using namespace bs_common;
using namespace beam_matching;

Eigen::Matrix4d PerturbPoseRandom(const Eigen::Matrix4d& T, double max_trans,
                                  double max_rot) {
  //   srand(time(NULL));
  Eigen::VectorXd perturb(6);
  perturb << beam::randf(max_rot, -max_rot), beam::randf(max_rot, -max_rot),
      beam::randf(max_rot, -max_rot), beam::randf(max_trans, -max_trans),
      beam::randf(max_trans, -max_trans), beam::randf(max_trans, -max_trans);
  return beam::PerturbTransformDegM(T, perturb);
}

class MultiScanRegistrationTest : public ::testing::Test {
protected:
  void SetUp() override {
    // read input cloud
    std::string current_file = "multi_scan_registration_tests.cpp";
    test_path_ = __FILE__;
    test_path_.erase(test_path_.end() - current_file.size(), test_path_.end());
    std::string scan_path = test_path_ + "data/test_scan_vlp16.pcd";
    PointCloudPtr test_cloud_tmp = std::make_shared<PointCloud>();
    PointCloud test_cloud;
    pcl::io::loadPCDFile(scan_path, *test_cloud_tmp);

    // create loam params from config
    std::string config_path = test_path_ + "data/loam_config.json";
    loam_params_ = std::make_shared<LoamParams>(config_path);

    // downsample input cloud
    Eigen::Vector3f scan_voxel_size(0.05, 0.05, 0.05);
    beam_filtering::VoxelDownsample<> downsampler(scan_voxel_size);
    downsampler.SetInputCloud(test_cloud_tmp);
    downsampler.Filter();
    test_cloud = downsampler.GetFilteredCloud();

    // create poses
    double max_pose_rot{20};
    double max_pose_trans{1};
    double max_pert_rot{10};
    double max_pert_trans{0.05};

    T_WORLD_S1_ = Eigen::Matrix4d::Identity();
    T_WORLD_S2_ = PerturbPoseRandom(T_WORLD_S1_, max_pose_trans, max_pose_rot);
    T_WORLD_S3_ = PerturbPoseRandom(T_WORLD_S2_, max_pose_trans, max_pose_rot);
    T_WORLD_S2_pert_ =
        PerturbPoseRandom(T_WORLD_S2_, max_pert_trans, max_pert_rot);
    T_S1_S2_ = beam::InvertTransform(T_WORLD_S1_) * T_WORLD_S2_;
    T_WORLD_S3_pert_ =
        PerturbPoseRandom(T_WORLD_S3_, max_pert_trans, max_pert_rot);
    T_S1_S3_ = beam::InvertTransform(T_WORLD_S1_) * T_WORLD_S3_;
    T_S2_S3_ = beam::InvertTransform(T_WORLD_S2_) * T_WORLD_S3_;

    // create scans
    S1_ = test_cloud;
    pcl::transformPointCloud(S1_, S2_, beam::InvertTransform(T_S1_S2_));
    pcl::transformPointCloud(S1_, S3_, beam::InvertTransform(T_S1_S3_));

    // create scan matcher params
    icp_params_.max_corr = 1;
    icp_params_.max_iter = 50;
    icp_params_.t_eps = 1e-8;
    icp_params_.fit_eps = 1e-2;
    icp_params_.lidar_ang_covar = 7.78e-9;
    icp_params_.lidar_lin_covar = 2.5e-4;
    icp_params_.multiscale_steps = 0;
    icp_params_.res = 0;

    scan_reg_params_.min_motion_trans_m = 0;
    scan_reg_params_.min_motion_rot_deg = 0;
    scan_reg_params_.fix_first_scan = true;
    scan_reg_params_.num_neighbors = 3;
    scan_reg_params_.lag_duration = 100;
    scan_reg_params_.disable_lidar_map = true;

    // create prior that will be used in optimization
    ScanPose SP_TMP(S1_, ros::Time(1), T_WORLD_S1_);

    fuse_core::Vector7d mean;
    mean << SP_TMP.Position().x(), SP_TMP.Position().y(), SP_TMP.Position().z(),
        SP_TMP.Orientation().w(), SP_TMP.Orientation().x(),
        SP_TMP.Orientation().y(), SP_TMP.Orientation().z();
    fuse_core::Matrix6d prior_covariance;
    prior_covariance.setIdentity();
    prior_covariance = prior_covariance * 1e-12;

    prior_ = fuse_constraints::AbsolutePose3DStampedConstraint(
        "PRIOR", SP_TMP.Position(), SP_TMP.Orientation(), mean,
        prior_covariance);

    // scan reg cov:
    covariance_.setIdentity();
    covariance_ = covariance_ * 0.1;
  }

  // void TearDown() override {}

  std::string test_path_;
  Eigen::Matrix4d T_WORLD_S1_;
  Eigen::Matrix4d T_WORLD_S2_;
  Eigen::Matrix4d T_WORLD_S2_pert_;
  Eigen::Matrix4d T_WORLD_S3_;
  Eigen::Matrix4d T_WORLD_S3_pert_;
  Eigen::Matrix4d T_S1_S2_;
  Eigen::Matrix4d T_S1_S3_;
  Eigen::Matrix4d T_S2_S3_;
  Eigen::Matrix4d T_BASELINK_LIDAR_ = Eigen::Matrix4d::Identity();
  PointCloud S1_;
  PointCloud S2_;
  PointCloud S3_;
  beam_matching::IcpMatcherParams icp_params_;
  std::shared_ptr<LoamParams> loam_params_;
  fuse_constraints::AbsolutePose3DStampedConstraint prior_;
  MultiScanRegistration::Params scan_reg_params_;
  Eigen::Matrix<double, 6, 6> covariance_;
};

int AddConstraints(const fuse_core::Transaction::SharedPtr& transaction,
                   fuse_graphs::HashGraph& graph) {
  int counter = 0;
  auto added_constraints = transaction->addedConstraints();
  for (auto iter = added_constraints.begin(); iter != added_constraints.end();
       iter++) {
    auto constraint =
        dynamic_cast<const fuse_constraints::RelativePose3DStampedConstraint&>(
            *iter);
    fuse_core::Constraint::SharedPtr constraint_ptr =
        fuse_constraints::RelativePose3DStampedConstraint::make_shared(
            constraint);
    graph.addConstraint(constraint_ptr);
    counter++;
  }
  return counter;
}

std::vector<fuse_core::UUID>
    AddVariables(const fuse_core::Transaction::SharedPtr& transaction,
                 fuse_graphs::HashGraph& graph) {
  fuse_variables::Position3DStamped dummy_position;
  fuse_variables::Orientation3DStamped dummy_orientation;
  std::vector<fuse_core::UUID> uuids;
  auto added_variables = transaction->addedVariables();
  for (auto iter = added_variables.begin(); iter != added_variables.end();
       iter++) {
    if (iter->type() == dummy_position.type()) {
      auto var = dynamic_cast<const fuse_variables::Position3DStamped&>(*iter);
      fuse_core::Variable::SharedPtr var_ptr =
          fuse_variables::Position3DStamped::make_shared(var);
      graph.addVariable(var_ptr);
    } else if (iter->type() == dummy_orientation.type()) {
      auto var =
          dynamic_cast<const fuse_variables::Orientation3DStamped&>(*iter);
      fuse_core::Variable::SharedPtr var_ptr =
          fuse_variables::Orientation3DStamped::make_shared(var);
      graph.addVariable(var_ptr);
    } else {
      return uuids;
    }
    uuids.push_back(iter->uuid());
  }
  return uuids;
}

bool VectorsEqual(double* v1, double* v2, int vsize) {
  double precision = 0.001;
  for (int i = 0; i < vsize; i++) {
    if (std::abs(v1[i] - v2[i]) > precision) { return false; }
  }
  return true;
}

TEST_F(MultiScanRegistrationTest, 2ScansManualConstraintAdding) {
  // create scan poses
  ScanPose SP1(S1_, ros::Time(1), T_WORLD_S1_);
  ScanPose SP2(S2_, ros::Time(2), T_WORLD_S2_);
  ScanPose SP2_pert(S2_, ros::Time(2), T_WORLD_S2_pert_);

  // init scan registration
  std::unique_ptr<beam_matching::IcpMatcher> matcher =
      std::make_unique<beam_matching::IcpMatcher>(icp_params_);

  MultiScanRegistration::Params scan_reg_params = scan_reg_params_;
  scan_reg_params.fix_first_scan = false;

  std::unique_ptr<MultiScanRegistration> multi_scan_registration =
      std::make_unique<MultiScanRegistration>(
          std::move(matcher), scan_reg_params.GetBaseParams(),
          scan_reg_params.num_neighbors, scan_reg_params.lag_duration,
          scan_reg_params.disable_lidar_map);
  multi_scan_registration->SetFixedCovariance(covariance_);

  auto transaction1 =
      multi_scan_registration->RegisterNewScan(SP1).GetTransaction();
  auto transaction2 =
      multi_scan_registration->RegisterNewScan(SP2_pert).GetTransaction();

  EXPECT_TRUE(transaction1 != nullptr);
  EXPECT_TRUE(transaction2 != nullptr);

  // validate stamps
  EXPECT_EQ(transaction1->stamp(), SP1.Stamp());
  EXPECT_EQ(transaction2->stamp(), SP2.Stamp());

  // Create the graph
  fuse_graphs::HashGraph graph;

  // add variables and validate uuids for each transaction
  std::vector<fuse_core::UUID> uuids;

  uuids = AddVariables(transaction1, graph);
  EXPECT_EQ(uuids.size(), 2);

  for (auto uuid : uuids) {
    bool should_be_true =
        uuid == SP1.Position().uuid() || uuid == SP1.Orientation().uuid();
    EXPECT_TRUE(should_be_true);
  }

  uuids = AddVariables(transaction2, graph);

  EXPECT_EQ(uuids.size(), 2);

  for (auto uuid : uuids) {
    bool should_be_true =
        uuid == SP2.Position().uuid() || uuid == SP2.Orientation().uuid();
    EXPECT_TRUE(should_be_true);
  }

  // add constraints and validate for each transaction
  int counter{0};
  counter += AddConstraints(transaction1, graph);
  counter += AddConstraints(transaction2, graph);
  EXPECT_EQ(counter, 1);

  // Add prior on first pose
  auto prior_ptr =
      std::make_shared<fuse_constraints::AbsolutePose3DStampedConstraint>(
          prior_);
  graph.addConstraint(prior_ptr);

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

  for (int i = 0; i < 3; i++) {
    EXPECT_TRUE(std::abs(SP1.Position().data()[i] - p1.data()[i]) < 0.001);
    EXPECT_TRUE(std::abs(SP2.Position().data()[i] - p2.data()[i]) < 0.001);
  }

  for (int i = 0; i < 4; i++) {
    EXPECT_TRUE(std::abs(SP1.Orientation().data()[i] - o1.data()[i]) < 0.001);
    EXPECT_TRUE(std::abs(SP2.Orientation().data()[i] - o2.data()[i]) < 0.001);
  }
}

TEST_F(MultiScanRegistrationTest, 3ScansManualConstraintAdding) {
  // create scan poses
  ScanPose SP1(S1_, ros::Time(1), T_WORLD_S1_);
  ScanPose SP2(S2_, ros::Time(2), T_WORLD_S2_);
  ScanPose SP3(S3_, ros::Time(3), T_WORLD_S3_);
  ScanPose SP2_pert(S2_, ros::Time(2), T_WORLD_S2_pert_);
  ScanPose SP3_pert(S3_, ros::Time(3), T_WORLD_S3_pert_);

  // init scan registration
  std::unique_ptr<beam_matching::IcpMatcher> matcher =
      std::make_unique<beam_matching::IcpMatcher>(icp_params_);

  MultiScanLoamRegistration::Params scan_reg_params = scan_reg_params_;
  scan_reg_params.fix_first_scan = false;

  std::unique_ptr<MultiScanRegistration> multi_scan_registration =
      std::make_unique<MultiScanRegistration>(
          std::move(matcher), scan_reg_params.GetBaseParams(),
          scan_reg_params.num_neighbors, scan_reg_params.lag_duration,
          scan_reg_params.disable_lidar_map);
  multi_scan_registration->SetFixedCovariance(covariance_);

  auto transaction1 =
      multi_scan_registration->RegisterNewScan(SP1).GetTransaction();
  auto transaction2 =
      multi_scan_registration->RegisterNewScan(SP2_pert).GetTransaction();
  auto transaction3 =
      multi_scan_registration->RegisterNewScan(SP3_pert).GetTransaction();

  // validate stamps
  EXPECT_EQ(transaction1->stamp(), SP1.Stamp());
  EXPECT_EQ(transaction2->stamp(), SP2.Stamp());
  EXPECT_EQ(transaction3->stamp(), SP3.Stamp());

  // Create the graph
  fuse_graphs::HashGraph graph;

  // add variables and validate uuids for each transaction
  std::vector<fuse_core::UUID> uuids;
  uuids = AddVariables(transaction1, graph);
  EXPECT_EQ(uuids.size(), 2);
  for (auto uuid : uuids) {
    bool should_be_true =
        uuid == SP1.Position().uuid() || uuid == SP1.Orientation().uuid();
    EXPECT_TRUE(should_be_true);
  }
  uuids = AddVariables(transaction2, graph);
  EXPECT_EQ(uuids.size(), 2);
  for (auto uuid : uuids) {
    bool should_be_true =
        uuid == SP2.Position().uuid() || uuid == SP2.Orientation().uuid();
    EXPECT_TRUE(should_be_true);
  }
  uuids = AddVariables(transaction3, graph);
  EXPECT_EQ(uuids.size(), 2);
  for (auto uuid : uuids) {
    bool should_be_true =
        uuid == SP3.Position().uuid() || uuid == SP3.Orientation().uuid();
    EXPECT_TRUE(should_be_true);
  }

  // add constraints and validate for each transaction
  int counter{0};
  counter += AddConstraints(transaction1, graph);
  counter += AddConstraints(transaction2, graph);
  counter += AddConstraints(transaction3, graph);
  EXPECT_EQ(counter, 3);

  // Add prior on first pose
  auto prior_ptr =
      std::make_shared<fuse_constraints::AbsolutePose3DStampedConstraint>(
          prior_);
  graph.addConstraint(prior_ptr);

  // Optimize the constraints and variables.
  graph.optimize();

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

  EXPECT_TRUE(VectorsEqual(SP1.Position().data(), p1.data(), 3));
  EXPECT_TRUE(VectorsEqual(SP1.Orientation().data(), o1.data(), 4));
  EXPECT_TRUE(VectorsEqual(SP2.Position().data(), p2.data(), 3));
  EXPECT_TRUE(VectorsEqual(SP2.Orientation().data(), o2.data(), 4));
  EXPECT_TRUE(VectorsEqual(SP3.Position().data(), p3.data(), 3));
  EXPECT_TRUE(VectorsEqual(SP3.Orientation().data(), o3.data(), 4));
}

TEST_F(MultiScanRegistrationTest, Params) {
  // init matchers
  std::unique_ptr<beam_matching::IcpMatcher> icp_matcher =
      std::make_unique<beam_matching::IcpMatcher>(icp_params_);

  std::shared_ptr<LoamParams> loam_params = std::make_shared<LoamParams>();
  std::unique_ptr<LoamMatcher> loam_matcher =
      std::make_unique<LoamMatcher>(*loam_params);
  std::shared_ptr<LoamFeatureExtractor> feature_extractor =
      std::make_shared<LoamFeatureExtractor>(loam_params);

  double min_motion_trans_m = 0.123;
  double min_motion_rot_deg = 0.456;
  double max_motion_trans_m = 10.123;
  bool fix_first_scan = true;
  int num_neighbors = 100;
  double lag_duration = 0.123;
  bool disable_lidar_map = true;

  MultiScanRegistrationBase::Params scan_reg_params;
  scan_reg_params.min_motion_trans_m = min_motion_trans_m;
  scan_reg_params.min_motion_rot_deg = min_motion_rot_deg;
  scan_reg_params.max_motion_trans_m = max_motion_trans_m;
  scan_reg_params.fix_first_scan = fix_first_scan;
  scan_reg_params.num_neighbors = num_neighbors;
  scan_reg_params.lag_duration = lag_duration;
  scan_reg_params.disable_lidar_map = disable_lidar_map;

  std::unique_ptr<MultiScanRegistration> multi_scan_icp_registration =
      std::make_unique<MultiScanRegistration>(
          std::move(icp_matcher), scan_reg_params.GetBaseParams(),
          scan_reg_params.num_neighbors, scan_reg_params.lag_duration,
          scan_reg_params.disable_lidar_map);

  MultiScanRegistration::Params returned_icp_multi_params =
      multi_scan_icp_registration->GetParams();
  EXPECT_EQ(returned_icp_multi_params.min_motion_trans_m, min_motion_trans_m);
  EXPECT_EQ(returned_icp_multi_params.min_motion_rot_deg, min_motion_rot_deg);
  EXPECT_EQ(returned_icp_multi_params.max_motion_trans_m, max_motion_trans_m);
  EXPECT_EQ(returned_icp_multi_params.fix_first_scan, fix_first_scan);
  EXPECT_EQ(returned_icp_multi_params.num_neighbors, num_neighbors);
  EXPECT_EQ(returned_icp_multi_params.lag_duration, lag_duration);
  EXPECT_EQ(returned_icp_multi_params.disable_lidar_map, disable_lidar_map);

  std::unique_ptr<MultiScanLoamRegistration> multi_scan_loam_registration =
      std::make_unique<MultiScanLoamRegistration>(
          std::move(loam_matcher), scan_reg_params.GetBaseParams(),
          scan_reg_params.num_neighbors, scan_reg_params.lag_duration,
          scan_reg_params.disable_lidar_map);

  MultiScanRegistration::Params returned_loam_multi_params =
      multi_scan_loam_registration->GetParams();
  EXPECT_EQ(returned_loam_multi_params.min_motion_trans_m, min_motion_trans_m);
  EXPECT_EQ(returned_loam_multi_params.min_motion_rot_deg, min_motion_rot_deg);
  EXPECT_EQ(returned_loam_multi_params.max_motion_trans_m, max_motion_trans_m);
  EXPECT_EQ(returned_loam_multi_params.fix_first_scan, fix_first_scan);
  EXPECT_EQ(returned_loam_multi_params.num_neighbors, num_neighbors);
  EXPECT_EQ(returned_loam_multi_params.lag_duration, lag_duration);
  EXPECT_EQ(returned_loam_multi_params.disable_lidar_map, disable_lidar_map);
}

TEST_F(MultiScanRegistrationTest, TransactionsAndUpdates) {
  // create scan poses
  ScanPose SP1(S1_, ros::Time(1), T_WORLD_S1_);
  ScanPose SP2(S2_, ros::Time(2), T_WORLD_S2_);
  ScanPose SP3(S3_, ros::Time(3), T_WORLD_S3_);
  ScanPose SP2_pert(S2_, ros::Time(2), T_WORLD_S2_pert_);
  ScanPose SP3_pert(S3_, ros::Time(3), T_WORLD_S3_pert_);

  // init scan registration
  std::unique_ptr<beam_matching::IcpMatcher> matcher =
      std::make_unique<beam_matching::IcpMatcher>(icp_params_);

  MultiScanRegistration::Params scan_reg_params = scan_reg_params_;
  std::cout << "scan_reg_params.fix_first_scan: "
            << scan_reg_params.fix_first_scan << "\n";
  std::cout << "scan_reg_params.GetBaseParams().fix_first_scan: "
            << scan_reg_params.GetBaseParams().fix_first_scan << "\n";
  scan_reg_params.disable_lidar_map = false;

  std::unique_ptr<MultiScanRegistration> multi_scan_registration =
      std::make_unique<MultiScanRegistration>(
          std::move(matcher), scan_reg_params.GetBaseParams(),
          scan_reg_params.num_neighbors, scan_reg_params.lag_duration,
          scan_reg_params.disable_lidar_map);
  multi_scan_registration->SetFixedCovariance(covariance_);

  // Create the graph
  fuse_graphs::HashGraph graph;

  // add transactions
  auto transaction1 =
      multi_scan_registration->RegisterNewScan(SP1).GetTransaction();
  graph.update(*transaction1);
  graph.optimize();
  EXPECT_TRUE(graph.variableExists(SP1.Position().uuid()));
  EXPECT_TRUE(!graph.variableExists(SP2_pert.Position().uuid()));
  EXPECT_TRUE(!graph.variableExists(SP3_pert.Position().uuid()));

  auto transaction2 =
      multi_scan_registration->RegisterNewScan(SP2_pert).GetTransaction();

  graph.update(*transaction2);
  graph.optimize();
  EXPECT_TRUE(graph.variableExists(SP1.Position().uuid()));
  EXPECT_TRUE(graph.variableExists(SP2_pert.Position().uuid()));
  EXPECT_TRUE(!graph.variableExists(SP3_pert.Position().uuid()));

  auto transaction3 =
      multi_scan_registration->RegisterNewScan(SP3_pert).GetTransaction();
  graph.update(*transaction3);
  graph.optimize();
  EXPECT_TRUE(graph.variableExists(SP1.Position().uuid()));
  EXPECT_TRUE(graph.variableExists(SP2_pert.Position().uuid()));
  EXPECT_TRUE(graph.variableExists(SP3_pert.Position().uuid()));

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

  //   multi_scan_registration->GetMap().Save("/userhome/tmp/loam_scan_registration/lidar_map2/");

  EXPECT_TRUE(VectorsEqual(SP1.Position().data(), p1.data(), 3));
  EXPECT_TRUE(VectorsEqual(SP1.Orientation().data(), o1.data(), 4));
  EXPECT_TRUE(VectorsEqual(SP2.Position().data(), p2.data(), 3));
  EXPECT_TRUE(VectorsEqual(SP2.Orientation().data(), o2.data(), 4));
  EXPECT_TRUE(VectorsEqual(SP3.Position().data(), p3.data(), 3));
  EXPECT_TRUE(VectorsEqual(SP3.Orientation().data(), o3.data(), 4));

  // test updating scan poses from graph msg
  auto SP1_ = multi_scan_registration->GetScan(SP1.Stamp());
  auto SP2_ = multi_scan_registration->GetScan(SP2.Stamp());
  auto SP3_ = multi_scan_registration->GetScan(SP3.Stamp());
  EXPECT_TRUE(SP1_.Updates() == 0);
  EXPECT_TRUE(!VectorsEqual(SP2.Position().data(), SP2_.Position().data(), 3));
  EXPECT_TRUE(
      !VectorsEqual(SP2.Orientation().data(), SP2_.Orientation().data(), 4));
  EXPECT_TRUE(SP2_.Updates() == 0);
  EXPECT_TRUE(!VectorsEqual(SP3.Position().data(), SP3_.Position().data(), 3));
  EXPECT_TRUE(
      !VectorsEqual(SP3.Orientation().data(), SP3_.Orientation().data(), 4));
  EXPECT_TRUE(SP3_.Updates() == 0);

  fuse_core::Graph::SharedPtr graph_ptr =
      fuse_graphs::HashGraph::make_shared(graph);
  multi_scan_registration->UpdateScanPoses(graph_ptr);
  auto SP1__ = multi_scan_registration->GetScan(SP1.Stamp());
  auto SP2__ = multi_scan_registration->GetScan(SP2.Stamp());
  auto SP3__ = multi_scan_registration->GetScan(SP3.Stamp());
  EXPECT_TRUE(VectorsEqual(SP1.Position().data(), SP1__.Position().data(), 3));
  EXPECT_TRUE(
      VectorsEqual(SP1.Orientation().data(), SP1__.Orientation().data(), 4));
  EXPECT_TRUE(SP1__.Updates() == 1);
  EXPECT_TRUE(VectorsEqual(SP2.Position().data(), SP2__.Position().data(), 3));
  EXPECT_TRUE(
      VectorsEqual(SP2.Orientation().data(), SP2__.Orientation().data(), 4));
  EXPECT_TRUE(SP2__.Updates() == 1);
  EXPECT_TRUE(VectorsEqual(SP3.Position().data(), SP3__.Position().data(), 3));
  EXPECT_TRUE(
      VectorsEqual(SP3.Orientation().data(), SP3__.Orientation().data(), 4));
  EXPECT_TRUE(SP3__.Updates() == 1);
}

TEST_F(MultiScanRegistrationTest, NumNeighbours) {
  // create scan poses
  ScanPose SP1(S1_, ros::Time(1), T_WORLD_S1_);
  ScanPose SP2(S2_, ros::Time(2), T_WORLD_S2_);
  ScanPose SP3(S3_, ros::Time(3), T_WORLD_S3_);
  ScanPose SP2_pert(S2_, ros::Time(2), T_WORLD_S2_pert_);
  ScanPose SP3_pert(S3_, ros::Time(3), T_WORLD_S3_pert_);

  // init scan registration
  std::unique_ptr<beam_matching::IcpMatcher> matcher1 =
      std::make_unique<beam_matching::IcpMatcher>(icp_params_);
  std::unique_ptr<beam_matching::IcpMatcher> matcher2 =
      std::make_unique<beam_matching::IcpMatcher>(icp_params_);

  auto scan_reg_params1 = scan_reg_params_;
  scan_reg_params1.num_neighbors = 1;

  std::unique_ptr<MultiScanRegistration> multi_scan_registration1 =
      std::make_unique<MultiScanRegistration>(
          std::move(matcher1), scan_reg_params1.GetBaseParams(),
          scan_reg_params1.num_neighbors, scan_reg_params1.lag_duration,
          scan_reg_params1.disable_lidar_map);

  auto scan_reg_params2 = scan_reg_params_;
  scan_reg_params1.num_neighbors = 2;

  std::unique_ptr<MultiScanRegistration> multi_scan_registration2 =
      std::make_unique<MultiScanRegistration>(
          std::move(matcher2), scan_reg_params2.GetBaseParams(),
          scan_reg_params2.num_neighbors, scan_reg_params2.lag_duration,
          scan_reg_params2.disable_lidar_map);
  multi_scan_registration1->SetFixedCovariance(covariance_);
  multi_scan_registration2->SetFixedCovariance(covariance_);

  // get transactions for each new scan
  auto transaction11 =
      multi_scan_registration1->RegisterNewScan(SP1).GetTransaction();
  auto transaction12 =
      multi_scan_registration1->RegisterNewScan(SP2_pert).GetTransaction();
  auto transaction13 =
      multi_scan_registration1->RegisterNewScan(SP3_pert).GetTransaction();

  auto transaction21 =
      multi_scan_registration2->RegisterNewScan(SP1).GetTransaction();
  auto transaction22 =
      multi_scan_registration2->RegisterNewScan(SP2_pert).GetTransaction();
  auto transaction23 =
      multi_scan_registration2->RegisterNewScan(SP3_pert).GetTransaction();

  // Create the graph and add transactions
  fuse_graphs::HashGraph graph1;
  EXPECT_TRUE(transaction11 != nullptr);
  graph1.update(*transaction11);

  EXPECT_TRUE(transaction12 != nullptr);
  graph1.update(*transaction12);

  EXPECT_TRUE(transaction13 != nullptr);
  graph1.update(*transaction13);

  fuse_graphs::HashGraph graph2;
  EXPECT_TRUE(transaction21 != nullptr);
  graph2.update(*transaction21);

  EXPECT_TRUE(transaction22 != nullptr);
  graph2.update(*transaction22);

  EXPECT_TRUE(transaction23 != nullptr);
  graph2.update(*transaction23);

  // check correct number of constraints are added
  auto constraints_added1 = graph1.getConstraints();
  int counter1 = 0;
  for (auto iter = constraints_added1.begin(); iter != constraints_added1.end();
       iter++) {
    counter1++;
  }
  // should be 1 for each scan
  EXPECT_TRUE(counter1 == 3);

  auto constraints_added2 = graph2.getConstraints();
  int counter2 = 0;
  for (auto iter = constraints_added2.begin(); iter != constraints_added2.end();
       iter++) {
    counter2++;
  }
  // should be one for each, except for the last one that will have 2
  EXPECT_TRUE(counter2 == 4);

  // Optimize the constraints and variables.
  graph1.optimize();
  graph2.optimize();

  auto p11 = dynamic_cast<const fuse_variables::Position3DStamped&>(
      graph1.getVariable(SP1.Position().uuid()));
  auto p12 = dynamic_cast<const fuse_variables::Position3DStamped&>(
      graph1.getVariable(SP2.Position().uuid()));
  auto p13 = dynamic_cast<const fuse_variables::Position3DStamped&>(
      graph1.getVariable(SP3.Position().uuid()));

  auto o11 = dynamic_cast<const fuse_variables::Orientation3DStamped&>(
      graph1.getVariable(SP1.Orientation().uuid()));
  auto o12 = dynamic_cast<const fuse_variables::Orientation3DStamped&>(
      graph1.getVariable(SP2.Orientation().uuid()));
  auto o13 = dynamic_cast<const fuse_variables::Orientation3DStamped&>(
      graph1.getVariable(SP3.Orientation().uuid()));

  EXPECT_TRUE(VectorsEqual(SP1.Position().data(), p11.data(), 3));
  EXPECT_TRUE(VectorsEqual(SP1.Orientation().data(), o11.data(), 4));
  EXPECT_TRUE(VectorsEqual(SP2.Position().data(), p12.data(), 3));
  EXPECT_TRUE(VectorsEqual(SP2.Orientation().data(), o12.data(), 4));
  EXPECT_TRUE(VectorsEqual(SP3.Position().data(), p13.data(), 3));
  EXPECT_TRUE(VectorsEqual(SP3.Orientation().data(), o13.data(), 4));

  auto p21 = dynamic_cast<const fuse_variables::Position3DStamped&>(
      graph2.getVariable(SP1.Position().uuid()));
  auto p22 = dynamic_cast<const fuse_variables::Position3DStamped&>(
      graph2.getVariable(SP2.Position().uuid()));
  auto p23 = dynamic_cast<const fuse_variables::Position3DStamped&>(
      graph2.getVariable(SP3.Position().uuid()));

  auto o21 = dynamic_cast<const fuse_variables::Orientation3DStamped&>(
      graph2.getVariable(SP1.Orientation().uuid()));
  auto o22 = dynamic_cast<const fuse_variables::Orientation3DStamped&>(
      graph2.getVariable(SP2.Orientation().uuid()));
  auto o23 = dynamic_cast<const fuse_variables::Orientation3DStamped&>(
      graph2.getVariable(SP3.Orientation().uuid()));

  EXPECT_TRUE(VectorsEqual(SP1.Position().data(), p21.data(), 3));
  EXPECT_TRUE(VectorsEqual(SP1.Orientation().data(), o21.data(), 4));
  EXPECT_TRUE(VectorsEqual(SP2.Position().data(), p22.data(), 3));
  EXPECT_TRUE(VectorsEqual(SP2.Orientation().data(), o22.data(), 4));
  EXPECT_TRUE(VectorsEqual(SP3.Position().data(), p23.data(), 3));
  EXPECT_TRUE(VectorsEqual(SP3.Orientation().data(), o23.data(), 4));
}

TEST_F(MultiScanRegistrationTest, RegistrationCases) {
  // create scan poses
  ScanPose SP1(S1_, ros::Time(1), T_WORLD_S1_);
  ScanPose SP2(S2_, ros::Time(2), T_WORLD_S2_);
  ScanPose SP3(S3_, ros::Time(3), T_WORLD_S3_);

  // init scan registration
  std::unique_ptr<beam_matching::IcpMatcher> matcher =
      std::make_unique<beam_matching::IcpMatcher>(icp_params_);

  auto scan_reg_params = scan_reg_params_;
  scan_reg_params.num_neighbors = 5;

  std::unique_ptr<MultiScanRegistration> multi_scan_registration =
      std::make_unique<MultiScanRegistration>(
          std::move(matcher), scan_reg_params.GetBaseParams(),
          scan_reg_params.num_neighbors, scan_reg_params.lag_duration,
          scan_reg_params.disable_lidar_map);
  multi_scan_registration->SetFixedCovariance(covariance_);

  // get transactions for each new scan
  auto transaction1 =
      multi_scan_registration->RegisterNewScan(SP1).GetTransaction();
  auto transaction2 =
      multi_scan_registration->RegisterNewScan(SP2).GetTransaction();
  auto transaction3 =
      multi_scan_registration->RegisterNewScan(SP3).GetTransaction();

  // Create the graph and add transactions
  fuse_graphs::HashGraph graph;
  graph.update(*transaction1);
  graph.update(*transaction2);
  graph.update(*transaction3);

  // create two bad ScanPose objects and make sure that no transactions are
  // added
  Eigen::VectorXd perturb(6);
  perturb << -45, 30, 90, 10, -10, 8;
  Eigen::Matrix4d T_WORLD_S4_pert =
      beam::PerturbTransformDegM(T_WORLD_S3_, perturb);
  ScanPose SP4_BADINIT(S3_, ros::Time(4), T_WORLD_S4_pert);
  ScanPose SP4_EMPTY(PointCloud(), ros::Time(4), T_WORLD_S4_pert);
  auto transactionNULL1 =
      multi_scan_registration->RegisterNewScan(SP4_BADINIT).GetTransaction();
  auto transactionNULL2 =
      multi_scan_registration->RegisterNewScan(SP4_EMPTY).GetTransaction();
  EXPECT_TRUE(transactionNULL1 == nullptr);
  EXPECT_TRUE(transactionNULL2 == nullptr);
}

TEST_F(MultiScanRegistrationTest, 2Scans) {
  // init scan registration
  std::unique_ptr<LoamMatcher> matcher =
      std::make_unique<LoamMatcher>(*loam_params_);
  std::shared_ptr<LoamFeatureExtractor> feature_extractor =
      std::make_shared<LoamFeatureExtractor>(loam_params_);

  // create scan poses
  ScanPose SP1(S1_, ros::Time(1), T_WORLD_S1_, T_BASELINK_LIDAR_,
               feature_extractor);
  ScanPose SP2(S2_, ros::Time(2), T_WORLD_S2_, T_BASELINK_LIDAR_,
               feature_extractor);
  ScanPose SP2_pert(S2_, ros::Time(2), T_WORLD_S2_pert_, T_BASELINK_LIDAR_,
                    feature_extractor);

  auto scan_reg_params = scan_reg_params_;
  scan_reg_params.num_neighbors = 1;
  std::unique_ptr<MultiScanLoamRegistration> multi_scan_registration =
      std::make_unique<MultiScanLoamRegistration>(
          std::move(matcher), scan_reg_params.GetBaseParams(),
          scan_reg_params.num_neighbors, scan_reg_params.lag_duration,
          scan_reg_params.disable_lidar_map);
  multi_scan_registration->SetFixedCovariance(covariance_);

  auto transaction1 =
      multi_scan_registration->RegisterNewScan(SP1).GetTransaction();
  auto transaction2 =
      multi_scan_registration->RegisterNewScan(SP2_pert).GetTransaction();

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

  EXPECT_TRUE(beam::ArePosesEqual(T_WORLD_S1_mea, T_WORLD_S1_, 1, 0.005, true));
  EXPECT_TRUE(beam::ArePosesEqual(T_WORLD_S2_mea, T_WORLD_S2_, 1, 0.05, true));
}

TEST_F(MultiScanRegistrationTest, 3Scans) {
  // init scan registration
  std::unique_ptr<LoamMatcher> matcher;
  matcher = std::make_unique<LoamMatcher>(*loam_params_);
  std::shared_ptr<LoamFeatureExtractor> feature_extractor =
      std::make_shared<LoamFeatureExtractor>(loam_params_);

  // create scan poses
  ScanPose SP1(S1_, ros::Time(1), T_WORLD_S1_, T_BASELINK_LIDAR_,
               feature_extractor);
  ScanPose SP2(S2_, ros::Time(2), T_WORLD_S2_, T_BASELINK_LIDAR_,
               feature_extractor);
  ScanPose SP3(S3_, ros::Time(3), T_WORLD_S3_, T_BASELINK_LIDAR_,
               feature_extractor);
  ScanPose SP2_pert(S2_, ros::Time(2), T_WORLD_S2_pert_, T_BASELINK_LIDAR_,
                    feature_extractor);
  ScanPose SP3_pert(S3_, ros::Time(3), T_WORLD_S3_pert_, T_BASELINK_LIDAR_,
                    feature_extractor);

  // init scan registration
  auto scan_reg_params = scan_reg_params_;
  scan_reg_params.disable_lidar_map = false;

  std::unique_ptr<MultiScanLoamRegistration> multi_scan_registration =
      std::make_unique<MultiScanLoamRegistration>(
          std::move(matcher), scan_reg_params.GetBaseParams(),
          scan_reg_params.num_neighbors, scan_reg_params.lag_duration,
          scan_reg_params.disable_lidar_map);

  // Create the graph
  std::shared_ptr<fuse_graphs::HashGraph> graph =
      std::make_shared<fuse_graphs::HashGraph>();

  // add transactions
  auto transaction1 =
      multi_scan_registration->RegisterNewScan(SP1).GetTransaction();
  auto transaction2 =
      multi_scan_registration->RegisterNewScan(SP2_pert).GetTransaction();
  auto transaction3 =
      multi_scan_registration->RegisterNewScan(SP3_pert).GetTransaction();

  graph->update(*transaction1);
  graph->update(*transaction2);
  graph->update(*transaction3);
  graph->optimize();

  SP1.UpdatePose(graph);
  SP2_pert.UpdatePose(graph);
  SP3_pert.UpdatePose(graph);

  // Check results
  auto p1 = dynamic_cast<const fuse_variables::Position3DStamped&>(
      graph->getVariable(SP1.Position().uuid()));
  auto p2 = dynamic_cast<const fuse_variables::Position3DStamped&>(
      graph->getVariable(SP2.Position().uuid()));
  auto p3 = dynamic_cast<const fuse_variables::Position3DStamped&>(
      graph->getVariable(SP3.Position().uuid()));

  auto o1 = dynamic_cast<const fuse_variables::Orientation3DStamped&>(
      graph->getVariable(SP1.Orientation().uuid()));
  auto o2 = dynamic_cast<const fuse_variables::Orientation3DStamped&>(
      graph->getVariable(SP2.Orientation().uuid()));
  auto o3 = dynamic_cast<const fuse_variables::Orientation3DStamped&>(
      graph->getVariable(SP3.Orientation().uuid()));

  Eigen::Matrix4d T_WORLD_S1_mea = bs_common::FusePoseToEigenTransform(p1, o1);
  Eigen::Matrix4d T_WORLD_S2_mea = bs_common::FusePoseToEigenTransform(p2, o2);
  Eigen::Matrix4d T_WORLD_S3_mea = bs_common::FusePoseToEigenTransform(p3, o3);

  // print lidar map
  //   RegistrationMap& map = RegistrationMap::GetInstance();
  //   map.Save("/userhome/tmp/loam_scan_registration/lidar_map/");

  //   EXPECT_TRUE(map.MapSize() == 3);
  EXPECT_TRUE(multi_scan_registration->GetMap().MapSize() == 3);

  EXPECT_TRUE(beam::ArePosesEqual(T_WORLD_S1_mea, T_WORLD_S1_, 1, 0.005, true));
  EXPECT_TRUE(beam::ArePosesEqual(T_WORLD_S2_mea, T_WORLD_S2_, 1, 0.05, true));
  EXPECT_TRUE(beam::ArePosesEqual(T_WORLD_S3_mea, T_WORLD_S3_, 1, 0.05, true));
}

TEST_F(MultiScanRegistrationTest, BaselinkLidarExtrinsics) {
  // init scan registration
  std::unique_ptr<LoamMatcher> matcher =
      std::make_unique<LoamMatcher>(*loam_params_);
  std::shared_ptr<LoamFeatureExtractor> feature_extractor =
      std::make_shared<LoamFeatureExtractor>(loam_params_);

  // perturb lidar extrinsics
  Eigen::Matrix4d T_BASELINK_LIDAR = T_WORLD_S1_ =
      PerturbPoseRandom(Eigen::Matrix4d::Identity(), 0.1, 30);

  // transform scans
  PointCloud S1 = S1_; // in lidar frame

  PointCloud S2;
  Eigen::Matrix4d T_LIDAR2_LIDAR1 = beam::InvertTransform(T_BASELINK_LIDAR) *
                                    beam::InvertTransform(T_WORLD_S2_) *
                                    T_WORLD_S1_ * T_BASELINK_LIDAR;
  pcl::transformPointCloud(S1, S2, T_LIDAR2_LIDAR1);

  PointCloud S3;
  Eigen::Matrix4d T_LIDAR3_LIDAR1 = beam::InvertTransform(T_BASELINK_LIDAR) *
                                    beam::InvertTransform(T_WORLD_S3_) *
                                    T_WORLD_S1_ * T_BASELINK_LIDAR;
  pcl::transformPointCloud(S1, S3, T_LIDAR3_LIDAR1);

  // create scan poses
  ScanPose SP1(S1, ros::Time(1), T_WORLD_S1_, T_BASELINK_LIDAR,
               feature_extractor);
  ScanPose SP2(S2, ros::Time(2), T_WORLD_S2_, T_BASELINK_LIDAR,
               feature_extractor);
  ScanPose SP3(S3, ros::Time(3), T_WORLD_S3_, T_BASELINK_LIDAR,
               feature_extractor);
  ScanPose SP2_pert(S2, ros::Time(2), T_WORLD_S2_pert_, T_BASELINK_LIDAR,
                    feature_extractor);
  ScanPose SP3_pert(S3, ros::Time(3), T_WORLD_S3_pert_, T_BASELINK_LIDAR,
                    feature_extractor);

  // init scan registration
  auto scan_reg_params = scan_reg_params_;
  scan_reg_params.disable_lidar_map = false;

  std::unique_ptr<MultiScanLoamRegistration> multi_scan_registration =
      std::make_unique<MultiScanLoamRegistration>(
          std::move(matcher), scan_reg_params.GetBaseParams(),
          scan_reg_params.num_neighbors, scan_reg_params.lag_duration,
          scan_reg_params.disable_lidar_map);

  // Create the graph
  std::shared_ptr<fuse_graphs::HashGraph> graph =
      std::make_shared<fuse_graphs::HashGraph>();

  // add transactions
  auto transaction1 =
      multi_scan_registration->RegisterNewScan(SP1).GetTransaction();
  auto transaction2 =
      multi_scan_registration->RegisterNewScan(SP2_pert).GetTransaction();
  auto transaction3 =
      multi_scan_registration->RegisterNewScan(SP3_pert).GetTransaction();

  graph->update(*transaction1);
  graph->update(*transaction2);
  graph->update(*transaction3);
  graph->optimize();

  SP1.UpdatePose(graph);
  SP2_pert.UpdatePose(graph);
  SP3_pert.UpdatePose(graph);

  //   SP1.SaveCloud("/userhome/tmp/loam_scan_registration/sp1/", true, true);
  //   SP2_pert.SaveCloud("/userhome/tmp/loam_scan_registration/sp2/", true,
  //                      true);
  //   SP3_pert.SaveCloud("/userhome/tmp/loam_scan_registration/sp3/", true,
  //                      true);

  // Check results
  auto p1 = dynamic_cast<const fuse_variables::Position3DStamped&>(
      graph->getVariable(SP1.Position().uuid()));
  auto p2 = dynamic_cast<const fuse_variables::Position3DStamped&>(
      graph->getVariable(SP2.Position().uuid()));
  auto p3 = dynamic_cast<const fuse_variables::Position3DStamped&>(
      graph->getVariable(SP3.Position().uuid()));

  auto o1 = dynamic_cast<const fuse_variables::Orientation3DStamped&>(
      graph->getVariable(SP1.Orientation().uuid()));
  auto o2 = dynamic_cast<const fuse_variables::Orientation3DStamped&>(
      graph->getVariable(SP2.Orientation().uuid()));
  auto o3 = dynamic_cast<const fuse_variables::Orientation3DStamped&>(
      graph->getVariable(SP3.Orientation().uuid()));

  Eigen::Matrix4d T_WORLD_S1_mea = bs_common::FusePoseToEigenTransform(p1, o1);
  Eigen::Matrix4d T_WORLD_S2_mea = bs_common::FusePoseToEigenTransform(p2, o2);
  Eigen::Matrix4d T_WORLD_S3_mea = bs_common::FusePoseToEigenTransform(p3, o3);

  EXPECT_TRUE(multi_scan_registration->GetMap().MapSize() == 3);

  EXPECT_TRUE(beam::ArePosesEqual(T_WORLD_S1_mea, T_WORLD_S1_, 1, 0.005, true));
  EXPECT_TRUE(
      beam::ArePosesEqual(T_WORLD_S2_mea, T_WORLD_S2_, 1.5, 0.15, true));
  EXPECT_TRUE(
      beam::ArePosesEqual(T_WORLD_S3_mea, T_WORLD_S3_, 1.5, 0.15, true));
}

TEST_F(MultiScanRegistrationTest, NScansWNoise) {
  // Setup scan registration
  std::shared_ptr<LoamParams> loam_params = std::make_shared<LoamParams>();
  *loam_params = *loam_params_;
  loam_params->max_correspondence_distance = 0.3;
  loam_params->convergence_criteria_translation_m = 0.001;
  loam_params->convergence_criteria_rotation_deg = 0.1;
  loam_params->max_correspondence_iterations = 20;
  loam_params->output_ceres_summary = false;
  loam_params->output_optimization_summary = false;
  loam_params->optimizer_params =
      beam_optimization::CeresParams(test_path_ + "data/ceres_config.json");

  std::unique_ptr<LoamMatcher> matcher =
      std::make_unique<LoamMatcher>(*loam_params);
  std::shared_ptr<LoamFeatureExtractor> feature_extractor =
      std::make_shared<LoamFeatureExtractor>(loam_params);

  MultiScanLoamRegistration::Params scan_reg_params;
  scan_reg_params.min_motion_trans_m = 0;
  scan_reg_params.min_motion_rot_deg = 0;
  scan_reg_params.fix_first_scan = true;
  scan_reg_params.num_neighbors = 1; // use 7
  scan_reg_params.lag_duration = 1000;
  scan_reg_params.disable_lidar_map = true;

  std::unique_ptr<MultiScanLoamRegistration> multi_scan_registration =
      std::make_unique<MultiScanLoamRegistration>(
          std::move(matcher), scan_reg_params.GetBaseParams(),
          scan_reg_params.num_neighbors, scan_reg_params.lag_duration,
          scan_reg_params.disable_lidar_map);

  // create path
  std::vector<Eigen::Vector3d, beam::AlignVec3d> nodes;
  nodes.push_back(Eigen::Vector3d(0, 0, 0));
  nodes.push_back(Eigen::Vector3d(2, -0.5, 0));
  nodes.push_back(Eigen::Vector3d(4, 0.5, 0));
  nodes.push_back(Eigen::Vector3d(6, 0.7, 0));
  beam::SimplePathGenerator path(nodes);

  // create a vec of N scan poses
  int num_scans = 2; // use 15
  ros::Time stamp_current = ros::Time(0);
  ros::Duration time_inc = ros::Duration(1); // increment by 1 s
  std::vector<ScanPose> scan_poses;
  std::vector<Eigen::Matrix4d, beam::AlignMat4d> gt_poses;
  for (size_t i = 0; i < num_scans; i++) {
    // get pose
    double interpolation_point =
        static_cast<double>(i) / static_cast<double>(num_scans);
    Eigen::Matrix4d T_WORLD_BASELINK = path.GetPose(interpolation_point);

    // perturb pose if not first pose
    Eigen::Matrix4d T_WORLD_BASELINK_pert;
    if (i == 0) {
      T_WORLD_BASELINK_pert = T_WORLD_BASELINK;
    } else {
      T_WORLD_BASELINK_pert = PerturbPoseRandom(T_WORLD_BASELINK, 0.05, 5);
    }

    // transform pointcloud & add noise
    PointCloud pc;
    pcl::transformPointCloud(
        S1_, pc, beam::InvertTransform(T_WORLD_BASELINK * T_BASELINK_LIDAR_));
    beam::AddNoiseToCloud(pc, 0.01, false);

    // create scan pose
    ScanPose SP(pc, stamp_current, T_WORLD_BASELINK_pert, T_BASELINK_LIDAR_,
                feature_extractor);
    scan_poses.push_back(SP);
    gt_poses.push_back(T_WORLD_BASELINK);
    stamp_current = stamp_current + time_inc;
  }

  // Create the graph
  std::shared_ptr<fuse_graphs::HashGraph> graph =
      std::make_shared<fuse_graphs::HashGraph>();

  // add run scan registration and add transactions
  for (auto& scan_pose : scan_poses) {
    // scan_pose.Print();
    auto transaction =
        multi_scan_registration->RegisterNewScan(scan_pose).GetTransaction();
    if (transaction != nullptr) { graph->update(*transaction); }
  }

  // optimize
  graph->optimize();

  //  Update poses
  for (size_t i = 0; i < num_scans; i++) {
    ScanPose& scan_pose = scan_poses.at(i);
    scan_pose.UpdatePose(graph);

    // save scans
    // scan_pose.SaveCloud("/userhome/tmp/multi_scan_tests/");
    // PointCloud pc;
    // pcl::transformPointCloud(scan_pose.Cloud(), pc, gt_poses.at(i));
    // PointCloudCol pc_col = beam::ColorPointCloud(pc, 0, 0, 255);

    // std::string filename = "/userhome/tmp/multi_scan_tests/" +
    //                        std::to_string(scan_pose.Stamp().toSec()) +
    //                        "_ground_truth.pcd";
    // pcl::io::savePCDFileASCII(filename, pc_col);
  }

  //  Check poses
  for (size_t i = 0; i < num_scans; i++) {
    const ScanPose& scan_pose = scan_poses.at(i);
    EXPECT_TRUE(beam::ArePosesEqual(scan_pose.T_REFFRAME_BASELINK(),
                                    gt_poses.at(i), 1, 0.03, true));
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  std::cout << "Starting ROS test, make sure you have a roscore going\n";
  ros::init(argc, argv, "scan_to_map_registration_test");
  bs_models::test::SetCalibrationParams();
  int ret = RUN_ALL_TESTS();
  ros::shutdown();
  return ret;
}
