#define CATCH_CONFIG_MAIN

#include <random>

#include <catch2/catch.hpp>
#include <fuse_constraints/absolute_pose_3d_stamped_constraint.h>
#include <fuse_core/constraint.h>
#include <fuse_core/uuid.h>
#include <fuse_core/variable.h>
#include <fuse_graphs/hash_graph.h>
#include <pcl/io/pcd_io.h>

#include <beam_utils/math.h>
#include <beam_utils/pointclouds.h>

#include <beam_common/sensor_proc.h>
#include <beam_models/frame_to_frame/scan_matcher_3d.h>

PointCloudPtr test_cloud;
bool setup{false};

void Setup() {
  if (setup) { return; }
  std::string current_file = "scan_matcher_3d_tests.cpp";
  std::string test_path = __FILE__;
  test_path.erase(test_path.end() - current_file.size(), test_path.end());
  std::string scan_path = test_path + "data/testscan.pcd";
  test_cloud = boost::make_shared<PointCloud>();
  pcl::io::loadPCDFile(scan_path, *test_cloud);
  setup = true;
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

TEST_CASE("Test simple 2 node FG") {
  Setup();

  // create poses
  Eigen::Matrix4d T_WORLD_S1 = Eigen::Matrix4d::Identity();
  Eigen::VectorXd perturb(6);
  perturb << 5, -10, 3, 2, 0.5, 0;
  Eigen::Matrix4d T_WORLD_S2 = beam::PerturbTransformDegM(T_WORLD_S1, perturb);
  Eigen::Matrix4d T_S1_S2 = beam::InvertTransform(T_WORLD_S1) * T_WORLD_S2;

  // create scans
  PointCloudPtr S1 = boost::make_shared<PointCloud>();
  PointCloudPtr S2 = boost::make_shared<PointCloud>();
  *S1 = *test_cloud;
  pcl::transformPointCloud(*S1, *S2, beam::InvertTransform(T_S1_S2));

  // create scan poses
  std::shared_ptr<beam_models::frame_to_frame::ScanPose> SP1 =
      std::make_shared<beam_models::frame_to_frame::ScanPose>(ros::Time(0),
                                                              T_WORLD_S1, S1);
  std::shared_ptr<beam_models::frame_to_frame::ScanPose> SP2 =
      std::make_shared<beam_models::frame_to_frame::ScanPose>(ros::Time(1),
                                                              T_WORLD_S2, S2);

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
  auto constraint1 = CreateConstraint(p1, o1, p2, o2, T_S1_S2);
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
  Setup();

  // create poses
  Eigen::Matrix4d T_WORLD_S1 = Eigen::Matrix4d::Identity();
  Eigen::VectorXd perturb(6);
  perturb << 5, -10, 3, 2, 0.5, 0;
  Eigen::Matrix4d T_WORLD_S2 = beam::PerturbTransformDegM(T_WORLD_S1, perturb);
  Eigen::VectorXd perturb2(6);
  perturb2 << 1, -2, 3, 0.05, 0.1, -0.03;
  Eigen::Matrix4d T_WORLD_S2_pert =
      beam::PerturbTransformDegM(T_WORLD_S2, perturb2);
  Eigen::Matrix4d T_S1_S2 = beam::InvertTransform(T_WORLD_S1) * T_WORLD_S2;

  // create scans
  PointCloudPtr S1 = boost::make_shared<PointCloud>();
  PointCloudPtr S2 = boost::make_shared<PointCloud>();
  *S1 = *test_cloud;
  pcl::transformPointCloud(*S1, *S2, beam::InvertTransform(T_S1_S2));

  // create scan poses
  std::shared_ptr<beam_models::frame_to_frame::ScanPose> SP1 =
      std::make_shared<beam_models::frame_to_frame::ScanPose>(ros::Time(0),
                                                              T_WORLD_S1, S1);
  std::shared_ptr<beam_models::frame_to_frame::ScanPose> SP2 =
      std::make_shared<beam_models::frame_to_frame::ScanPose>(ros::Time(1),
                                                              T_WORLD_S2, S2);

  std::shared_ptr<beam_models::frame_to_frame::ScanPose> SP2_pert =
      std::make_shared<beam_models::frame_to_frame::ScanPose>(
          ros::Time(1), T_WORLD_S2_pert, S2);

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
  auto constraint1 = CreateConstraint(p1, o1, p2, o2, T_S1_S2);
  graph.addConstraint(constraint1);

  // Add prior on first pose
  fuse_core::Vector7d mean;
  mean << SP1->Position()->x(), SP1->Position()->y(), SP1->Position()->z(),
      SP1->Orientation()->w(), SP1->Orientation()->x(), SP1->Orientation()->y(),
      SP1->Orientation()->z();
  fuse_core::Matrix6d covariance;
  covariance.setIdentity();
  auto prior = fuse_constraints::AbsolutePose3DStampedConstraint(
      "PRIOR", *SP1->Position(), *SP1->Orientation(), mean, covariance);

  std::cout << "Original: ";
  p1->print();
  o1->print();
  p2->print();
  o2->print();

  // Optimize the constraints and variables.
  graph.optimize();

  std::cout << "Optimized:\n ";
  p1->print();
  o1->print();
  p2->print();
  o2->print();

  for (int i = 0; i < 3; i++) {
    REQUIRE(std::abs(SP1->Position()->data()[i] - p1->data()[i]) < 0.1);
    REQUIRE(std::abs(SP2->Position()->data()[i] - p2->data()[i]) < 0.1);
  }
  for (int i = 0; i < 4; i++) {
    REQUIRE(std::abs(SP1->Orientation()->data()[i] - o1->data()[i]) < 0.1);
    REQUIRE(std::abs(SP2->Orientation()->data()[i] - o2->data()[i]) < 0.1);
  }
}

TEST_CASE("Test simple 3 node FG with scan registration"){
  Setup();
}