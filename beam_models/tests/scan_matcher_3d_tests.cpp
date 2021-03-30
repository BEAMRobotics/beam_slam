#define CATCH_CONFIG_MAIN

#include <random>

#include <catch2/catch.hpp>
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

#include <beam_common/sensor_proc.h>
#include <beam_models/frame_to_frame/scan_matcher_3d.h>

class Data {
public:
  Data() {
    // read input cloud
    std::string current_file = "scan_matcher_3d_tests.cpp";
    std::string test_path = __FILE__;
    test_path.erase(test_path.end() - current_file.size(), test_path.end());
    std::string scan_path = test_path + "data/testscan.pcd";
    PointCloudPtr test_cloud_tmp = boost::make_shared<PointCloud>();
    PointCloudPtr test_cloud = boost::make_shared<PointCloud>();
    pcl::io::loadPCDFile(scan_path, *test_cloud_tmp);

    // downsample input cloud
    Eigen::Vector3f scan_voxel_size(0.05, 0.05, 0.05);
    beam_filtering::VoxelDownsample downsampler(scan_voxel_size);
    downsampler.Filter(*test_cloud_tmp, *test_cloud);

    // create poses
    T_WORLD_S1 = Eigen::Matrix4d::Identity();
    Eigen::VectorXd perturb(6);
    perturb << 5, -10, 3, 2, 0.5, 0;
    T_WORLD_S2 = beam::PerturbTransformDegM(T_WORLD_S1, perturb);
    Eigen::VectorXd perturb2(6);
    perturb2 << 1, -2, 3, 0.05, 0.1, -0.03;
    T_WORLD_S2_pert = beam::PerturbTransformDegM(T_WORLD_S2, perturb2);
    T_S1_S2 = beam::InvertTransform(T_WORLD_S1) * T_WORLD_S2;

    // create scans
    S1 = boost::make_shared<PointCloud>();
    S2 = boost::make_shared<PointCloud>();
    *S1 = *test_cloud;
    pcl::transformPointCloud(*S1, *S2, beam::InvertTransform(T_S1_S2));
  }

  Eigen::Matrix4d T_WORLD_S1;
  Eigen::Matrix4d T_WORLD_S2;
  Eigen::Matrix4d T_WORLD_S2_pert;
  Eigen::Matrix4d T_S1_S2;
  PointCloudPtr S1;
  PointCloudPtr S2;
};

Data data_;

void OutputTransformInformation(const Eigen::Matrix4d& T,
                                const std::string& transform_name) {
  Eigen::Matrix3d R = T.block(0, 0, 3, 3);
  Eigen::Vector3d rpy = R.eulerAngles(0, 1, 2);
  std::cout << transform_name << ":\n"
            << "R: \n"
            << R << "\n"
            << "t: [" << T(0, 3) << ", " << T(1, 3) << ", " << T(2, 3) << "]\n"
            << "rpy (deg): [" << beam::Rad2Deg(beam::WrapToPi(rpy[0])) << ", "
            << beam::Rad2Deg(beam::WrapToPi(rpy[1])) << ", "
            << beam::Rad2Deg(beam::WrapToPi(rpy[2])) << "]\n";
}

/**
 * @brief Dummy loss implementation for testing
 */
class ExampleLoss : public fuse_core::Loss {
public:
  FUSE_LOSS_DEFINITIONS(ExampleLoss);

  explicit ExampleLoss(const double a = 1.0) : a(a) {}

  void initialize(const std::string& /*name*/) override {}

  void print(std::ostream& /*stream = std::cout*/) const override {}

  ceres::LossFunction* lossFunction() const override {
    return new ceres::HuberLoss(a);
  }

  double a{1.0}; //!< Public member variable just for testing

private:
  // Allow Boost Serialization access to private methods
  friend class boost::serialization::access;

  /**
   * @brief The Boost Serialize method that serializes all of the data members
   * in to/out of the archive
   *
   * @param[in/out] archive - The archive object that holds the serialized class
   * members
   * @param[in] version - The version of the archive being read/written.
   * Generally unused.
   */
  template <class Archive>
  void serialize(Archive& archive, const unsigned int /* version */) {
    archive& boost::serialization::base_object<fuse_core::Loss>(*this);
    archive& a;
  }
};

fuse_constraints::RelativePose3DStampedConstraint::SharedPtr CreateConstraint(
    const fuse_variables::Position3DStamped::SharedPtr& position1,
    const fuse_variables::Orientation3DStamped::SharedPtr& orientation1,
    const fuse_variables::Position3DStamped::SharedPtr& position2,
    const fuse_variables::Orientation3DStamped::SharedPtr& orientation2,
    const Eigen::Matrix4d& T_CLOUD1_CLOUD2) {
  // convert rotation matrix to quaternion
  Eigen::Matrix3d R = T_CLOUD1_CLOUD2.block(0, 0, 3, 3);
  Eigen::Quaterniond q(R);

  // Convert measurement to fuse variable
  fuse_core::Vector7d pose_relative_mean;
  pose_relative_mean << T_CLOUD1_CLOUD2(0, 3), T_CLOUD1_CLOUD2(1, 3),
      T_CLOUD1_CLOUD2(2, 3), q.w(), q.x(), q.y(), q.z();

  // create identity covariance
  Eigen::Matrix<double, 6, 6> covariance;
  covariance.setIdentity();
  // covariance = covariance * 0.1;

  // Create a relative pose constraint. We assume the pose measurements are
  // independent.
  auto constraint =
      fuse_constraints::RelativePose3DStampedConstraint::make_shared(
          "SOURCE", *position1, *orientation1, *position2, *orientation2,
          pose_relative_mean, covariance);
  return constraint;
}

TEST_CASE("Test scan pose") {
  // create scan poses
  std::shared_ptr<beam_models::frame_to_frame::ScanPose> SP1 =
      std::make_shared<beam_models::frame_to_frame::ScanPose>(
          ros::Time(0), data_.T_WORLD_S1, data_.S1);
  std::shared_ptr<beam_models::frame_to_frame::ScanPose> SP2 =
      std::make_shared<beam_models::frame_to_frame::ScanPose>(
          ros::Time(1), data_.T_WORLD_S2, data_.S2);

  Eigen::Matrix4d T_WORLD_S2_ = SP2->T_WORLD_CLOUD();
  Eigen::Matrix4d T_WORLD_S1_ = SP1->T_WORLD_CLOUD();
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      REQUIRE(data_.T_WORLD_S1(i, j) ==
              Approx(T_WORLD_S1_(i, j)).epsilon(0.0001));
      REQUIRE(data_.T_WORLD_S2(i, j) ==
              Approx(T_WORLD_S2_(i, j)).epsilon(0.0001));
    }
  }
  REQUIRE(SP1->Cloud()->size() == data_.S1->size());
  REQUIRE(SP2->Cloud()->size() == data_.S2->size());
  REQUIRE(SP1->Stamp() == ros::Time(0));
  REQUIRE(SP2->Stamp() == ros::Time(1));
}

TEST_CASE("Test simple 2 node FG") {
  // create scan poses
  std::shared_ptr<beam_models::frame_to_frame::ScanPose> SP1 =
      std::make_shared<beam_models::frame_to_frame::ScanPose>(
          ros::Time(0), data_.T_WORLD_S1, data_.S1);
  std::shared_ptr<beam_models::frame_to_frame::ScanPose> SP2 =
      std::make_shared<beam_models::frame_to_frame::ScanPose>(
          ros::Time(1), data_.T_WORLD_S2, data_.S2);

  // Create the graph
  fuse_graphs::HashGraph graph;

  // Add variables
  fuse_variables::Position3DStamped::SharedPtr p1 = SP1->Position();
  fuse_variables::Orientation3DStamped::SharedPtr o1 = SP1->Orientation();
  fuse_variables::Position3DStamped::SharedPtr p2 = SP2->Position();
  fuse_variables::Orientation3DStamped::SharedPtr o2 = SP2->Orientation();
  graph.addVariable(p1);
  graph.addVariable(o1);
  graph.addVariable(p2);
  graph.addVariable(o2);

  // Create loss
  auto loss = ExampleLoss::make_shared();

  // Add constraint
  auto constraint1 = CreateConstraint(p1, o1, p2, o2, data_.T_S1_S2);
  graph.addConstraint(constraint1);

  // Optimize the constraints and variables.
  graph.optimize();

  for (int i = 0; i < 3; i++) {
    REQUIRE(p1->data()[i] == SP1->Position()->data()[i]);
    REQUIRE(p2->data()[i] == SP2->Position()->data()[i]);
  }
  for (int i = 0; i < 4; i++) {
    REQUIRE(o1->data()[i] == SP1->Orientation()->data()[i]);
    REQUIRE(o2->data()[i] == SP2->Orientation()->data()[i]);
  }
}

TEST_CASE("Test simple 2 node FG with perturbed pose") {
  // create scan poses
  std::shared_ptr<beam_models::frame_to_frame::ScanPose> SP1 =
      std::make_shared<beam_models::frame_to_frame::ScanPose>(
          ros::Time(0), data_.T_WORLD_S1, data_.S1);
  std::shared_ptr<beam_models::frame_to_frame::ScanPose> SP2 =
      std::make_shared<beam_models::frame_to_frame::ScanPose>(
          ros::Time(1), data_.T_WORLD_S2, data_.S2);

  std::shared_ptr<beam_models::frame_to_frame::ScanPose> SP2_pert =
      std::make_shared<beam_models::frame_to_frame::ScanPose>(
          ros::Time(1), data_.T_WORLD_S2_pert, data_.S2);

  // Create the graph
  fuse_graphs::HashGraph graph;

  // Add variables
  fuse_variables::Position3DStamped::SharedPtr p1 = SP1->Position();
  fuse_variables::Orientation3DStamped::SharedPtr o1 = SP1->Orientation();
  fuse_variables::Position3DStamped::SharedPtr p2 = SP2_pert->Position();
  fuse_variables::Orientation3DStamped::SharedPtr o2 = SP2_pert->Orientation();
  graph.addVariable(p1);
  graph.addVariable(o1);
  graph.addVariable(p2);
  graph.addVariable(o2);

  // Create loss
  auto loss = ExampleLoss::make_shared();

  // Add constraint between poses
  auto constraint1 = CreateConstraint(p1, o1, p2, o2, data_.T_S1_S2);
  graph.addConstraint(constraint1);

  // Add prior on first pose
  fuse_core::Vector7d mean;
  mean << SP1->Position()->x(), SP1->Position()->y(), SP1->Position()->z(),
      SP1->Orientation()->w(), SP1->Orientation()->x(), SP1->Orientation()->y(),
      SP1->Orientation()->z();
  fuse_core::Matrix6d covariance;
  covariance.setIdentity();
  covariance = covariance * 0.0000000001;
  auto prior =
      std::make_shared<fuse_constraints::AbsolutePose3DStampedConstraint>(
          "PRIOR", *SP1->Position(), *SP1->Orientation(), mean, covariance);
  graph.addConstraint(prior);

  // Optimize the constraints and variables.
  graph.optimize();

  for (int i = 0; i < 3; i++) {
    REQUIRE(std::abs(SP1->Position()->data()[i] - p1->data()[i]) < 0.001);
    REQUIRE(std::abs(SP2->Position()->data()[i] - p2->data()[i]) < 0.001);
  }
  for (int i = 0; i < 4; i++) {
    REQUIRE(std::abs(SP1->Orientation()->data()[i] - o1->data()[i]) < 0.001);
    REQUIRE(std::abs(SP2->Orientation()->data()[i] - o2->data()[i]) < 0.001);
  }
}

TEST_CASE("Test scan registration - vanilla icp") {
  // create scan poses
  std::shared_ptr<beam_models::frame_to_frame::ScanPose> SP1 =
      std::make_shared<beam_models::frame_to_frame::ScanPose>(
          ros::Time(0), data_.T_WORLD_S1, data_.S1);
  std::shared_ptr<beam_models::frame_to_frame::ScanPose> SP2 =
      std::make_shared<beam_models::frame_to_frame::ScanPose>(
          ros::Time(1), data_.T_WORLD_S2, data_.S2);

  std::shared_ptr<beam_models::frame_to_frame::ScanPose> SP2_pert =
      std::make_shared<beam_models::frame_to_frame::ScanPose>(
          ros::Time(1), data_.T_WORLD_S2_pert, data_.S2);

  Eigen::Matrix4d T_S1_S2_initial =
      beam::InvertTransform(data_.T_WORLD_S1) * data_.T_WORLD_S2_pert;
  PointCloudPtr S2_RefFEst = boost::make_shared<PointCloud>();
  pcl::transformPointCloud(*(SP2_pert->Cloud()), *S2_RefFEst,
                           Eigen::Affine3d(T_S1_S2_initial));

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> matcher;
  matcher.setInputSource(S2_RefFEst);
  matcher.setInputTarget(SP1->Cloud());
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
  pcl::transformPointCloud(*SP2_pert->Cloud(), S2_RefFOpt2, T_S1_S2_opt);

  // output results
  /*
  pcl::io::savePCDFileASCII("/home/nick/tmp/S1_RefF.pcd", *SP1->Cloud());
  pcl::io::savePCDFileASCII("/home/nick/tmp/S2_RefFEst.pcd", *S2_RefFEst);
  pcl::io::savePCDFileASCII("/home/nick/tmp/S2_RefFOpt2.pcd", S2_RefFOpt2);
  pcl::io::savePCDFileASCII("/home/nick/tmp/S2_RefFOpt1.pcd", S2_RefFOpt1);
  */

  // check final transform is close to original
  REQUIRE(data_.T_S1_S2.norm() == Approx(T_S1_S2_opt.norm()).epsilon(0.0001));
}

TEST_CASE("Test simple 2 node FG with perturbed pose and scan registration") {
  // create scan poses
  std::shared_ptr<beam_models::frame_to_frame::ScanPose> SP1 =
      std::make_shared<beam_models::frame_to_frame::ScanPose>(
          ros::Time(0), data_.T_WORLD_S1, data_.S1);
  std::shared_ptr<beam_models::frame_to_frame::ScanPose> SP2 =
      std::make_shared<beam_models::frame_to_frame::ScanPose>(
          ros::Time(1), data_.T_WORLD_S2, data_.S2);

  std::shared_ptr<beam_models::frame_to_frame::ScanPose> SP2_pert =
      std::make_shared<beam_models::frame_to_frame::ScanPose>(
          ros::Time(1), data_.T_WORLD_S2_pert, data_.S2);

  // run scan registration
  // * (1) transform second scan into estimated scan 1 frame
  // * (2) run scan macher
  // * (3) get estimated transform between two frames
  //
  Eigen::Matrix4d T_S1_S2_init =
      beam::InvertTransform(data_.T_WORLD_S1) * data_.T_WORLD_S2_pert;
  PointCloudPtr S2_RefFEst = boost::make_shared<PointCloud>();
  pcl::transformPointCloud(*(SP2_pert->Cloud()), *S2_RefFEst,
                           Eigen::Affine3d(T_S1_S2_init));

  beam_matching::IcpMatcherParams matcher_params;
  matcher_params.max_corr = 1;
  matcher_params.max_iter = 50;
  matcher_params.t_eps = 1e-8;
  matcher_params.fit_eps = 1e-2;
  matcher_params.lidar_ang_covar = 7.78e-9;
  matcher_params.lidar_lin_covar = 2.5e-4;
  matcher_params.multiscale_steps = 0;
  matcher_params.res = 0;

  std::unique_ptr<beam_matching::IcpMatcher> matcher =
      std::make_unique<beam_matching::IcpMatcher>(matcher_params);
  matcher->SetRef(S2_RefFEst);
  matcher->SetTarget(SP1->Cloud());
  matcher->Match();
  matcher->EstimateInfo();

  Eigen::Matrix4d T_S1Opt_S1Ini = matcher->GetResult().matrix();
  Eigen::Matrix4d T_S1_S2_opt = T_S1Opt_S1Ini * T_S1_S2_init;
  Eigen::Matrix<double, 6, 6> covariance = matcher->GetInfo();

  Eigen::Matrix4d T_W_S2_opt = data_.T_WORLD_S1 * T_S1_S2_opt;

  // output results
  /*
  OutputTransformInformation(T_S1_S2_init, "T_S1_S2_init");
  OutputTransformInformation(T_S1_S2_opt, "T_S1_S2_opt");
  OutputTransformInformation(data_.T_S1_S2, "T_S1_S2_gt");

  
  PointCloud S1_w_gt;
  PointCloud S2_w_gt;
  PointCloud S2_w_init;
  PointCloud S2_w_opt;

  Eigen::Matrix4d T_W_S2_gt = data_.T_WORLD_S1 * data_.T_S1_S2; 
  Eigen::Matrix4d T_W_S2_init = data_.T_WORLD_S1 * T_S1_S2_init;
  
    
  pcl::transformPointCloud(*SP1->Cloud(), S1_w_gt, data_.T_WORLD_S1);
  pcl::transformPointCloud(*SP2_pert->Cloud(), S2_w_gt, T_W_S2_gt);
  pcl::transformPointCloud(*SP2_pert->Cloud(), S2_w_init, T_W_S2_init);
  pcl::transformPointCloud(*SP2_pert->Cloud(), S2_w_opt, T_W_S2_opt);

  pcl::io::savePCDFileASCII("/home/nick/tmp/S1_w_gt.pcd", S1_w_gt);
  pcl::io::savePCDFileASCII("/home/nick/tmp/S2_w_gt.pcd", S2_w_gt);
  pcl::io::savePCDFileASCII("/home/nick/tmp/S2_w_init.pcd", S2_w_init);
  pcl::io::savePCDFileASCII("/home/nick/tmp/S2_w_opt.pcd", S2_w_opt);
  */

  // check final transform is close to original
  REQUIRE(data_.T_S1_S2.norm() == Approx(T_S1_S2_opt.norm()).epsilon(0.0001));
  REQUIRE(data_.T_WORLD_S2.norm() == Approx(T_W_S2_opt.norm()).epsilon(0.0001));
  
  // Create the graph
  fuse_graphs::HashGraph graph;

  // Add variables
  fuse_variables::Position3DStamped::SharedPtr p1 = SP1->Position();
  fuse_variables::Orientation3DStamped::SharedPtr o1 = SP1->Orientation();
  fuse_variables::Position3DStamped::SharedPtr p2 = SP2_pert->Position();
  fuse_variables::Orientation3DStamped::SharedPtr o2 = SP2_pert->Orientation();
  graph.addVariable(p1);
  graph.addVariable(o1);
  graph.addVariable(p2);
  graph.addVariable(o2);

  // Create loss
  auto loss = ExampleLoss::make_shared();

  // Add constraint between poses
  auto constraint1 = CreateConstraint(p1, o1, p2, o2, T_S1_S2_opt);
  graph.addConstraint(constraint1);

  // Add prior on first pose
  fuse_core::Vector7d mean;
  mean << SP1->Position()->x(), SP1->Position()->y(), SP1->Position()->z(),
      SP1->Orientation()->w(), SP1->Orientation()->x(), SP1->Orientation()->y(),
      SP1->Orientation()->z();
  fuse_core::Matrix6d prior_covariance;
  prior_covariance.setIdentity();
  prior_covariance = prior_covariance * 0.0000000001;
  auto prior =
      std::make_shared<fuse_constraints::AbsolutePose3DStampedConstraint>(
          "PRIOR", *SP1->Position(), *SP1->Orientation(), mean,
          prior_covariance);
  graph.addConstraint(prior);

  // Optimize the constraints and variables.
  graph.optimize();

  for (int i = 0; i < 3; i++) {
    REQUIRE(std::abs(SP1->Position()->data()[i] - p1->data()[i]) < 0.001);
    REQUIRE(std::abs(SP2->Position()->data()[i] - p2->data()[i]) < 0.001);
  }
  for (int i = 0; i < 4; i++) {
    REQUIRE(std::abs(SP1->Orientation()->data()[i] - o1->data()[i]) < 0.001);
    REQUIRE(std::abs(SP2->Orientation()->data()[i] - o2->data()[i]) < 0.001);
  }
}

