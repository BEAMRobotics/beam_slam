#include <gtest/gtest.h>

#include <pcl/io/pcd_io.h>

#include <fuse_graphs/hash_graph.h>

#include <beam_filtering/VoxelDownsample.h>
#include <beam_matching/Matchers.h>
#include <beam_utils/math.h>
#include <beam_utils/pointclouds.h>
#include <beam_utils/simple_path_generator.h>

#include <bs_models/scan_registration/multi_scan_registration.h>
#include <bs_models/scan_registration/scan_to_map_registration.h>
#include <bs_models/global_mapping/global_map.h>
#include <bs_models/global_mapping/global_map_refinement.h>

using namespace bs_models;
using namespace global_mapping;
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

class GlobalMapRefinementTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // get data path
    std::string current_file = "global_map_refinement_tests.cpp";
    test_path_ = __FILE__;
    test_path_.erase(test_path_.end() - current_file.size(), test_path_.end());

    // read input cloud
    std::string scan_path = test_path_ + "data/test_scan_vlp16.pcd";
    PointCloud cloud_tmp;
    pcl::io::loadPCDFile(scan_path, cloud_tmp);

    // get path to configs/data files
    extrinsics_path_ = test_path_ + "data/extrinsics.json";
    frame_ids_path_ = test_path_ + "data/frame_ids.json";
    refinement_config_path_ = test_path_ + "data/global_map_refinement.json";

    // downsample input cloud
    Eigen::Vector3f scan_voxel_size(0.05, 0.05, 0.05);
    beam_filtering::VoxelDownsample<> downsampler(scan_voxel_size);
    downsampler.SetInputCloud(std::make_shared<PointCloud>(cloud_tmp));
    downsampler.Filter();
    PointCloud cloud_in_lidar_frame = downsampler.GetFilteredCloud();

    // create scan matcher params
    loam_params_ = std::make_shared<LoamParams>();
    loam_params_->max_correspondence_distance = 0.3;
    loam_params_->convergence_criteria_translation_m = 0.001;
    loam_params_->convergence_criteria_rotation_deg = 0.1;
    loam_params_->max_correspondence_iterations = 20;
    loam_params_->output_ceres_summary = false;
    loam_params_->output_optimization_summary = false;
    loam_params_->optimizer_params = beam_optimization::CeresParams(
        test_path_ + "data/ceres_config_refinement.json");

    feature_extractor_ = std::make_shared<LoamFeatureExtractor>(loam_params_);

    // scan reg cov:
    covariance_.setIdentity();
    covariance_ = covariance_ * 0.1;

    // create extrinsics
    extrinsics_ = std::make_shared<ExtrinsicsLookupBase>(frame_ids_path_,
                                                         extrinsics_path_);
    extrinsics_->GetT_BASELINK_LIDAR(T_BASELINK_LIDAR_);
    pcl::transformPointCloud(cloud_in_lidar_frame, cloud_in_world_frame_,
                             T_BASELINK_LIDAR_);
  }

  // void TearDown() override {}

  std::string test_path_;
  std::string extrinsics_path_;
  std::string frame_ids_path_;
  std::string refinement_config_path_;

  std::shared_ptr<ExtrinsicsLookupBase> extrinsics_;
  Eigen::Matrix4d T_BASELINK_LIDAR_;
  PointCloud cloud_in_world_frame_;

  std::shared_ptr<LoamParams> loam_params_;
  std::shared_ptr<LoamFeatureExtractor> feature_extractor_;
  Eigen::Matrix<double, 6, 6> covariance_;

  std::string output_root_ = "/home/nick/tmp/refinement_tests/";
};

TEST_F(GlobalMapRefinementTest, MultiScan) {
  // create path
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> nodes;
  nodes.push_back(Eigen::Vector3d(0, 0, 0));
  nodes.push_back(Eigen::Vector3d(2, -0.5, 0));
  nodes.push_back(Eigen::Vector3d(4, 0.5, 0));
  nodes.push_back(Eigen::Vector3d(6, 0.7, 0));
  beam::SimplePathGenerator path(nodes);

  // create scan poses
  int num_scans = 15;
  ros::Time stamp_current = ros::Time(0);
  ros::Duration time_inc = ros::Duration(1);  // increment by 1 s
  std::vector<ScanPose> scan_poses;
  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>
      gt_poses;
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
    PointCloud cloud_in_lidar_frame;
    pcl::transformPointCloud(
        cloud_in_world_frame_, cloud_in_lidar_frame,
        beam::InvertTransform(T_WORLD_BASELINK * T_BASELINK_LIDAR_));
    beam::AddNoiseToCloud(cloud_in_lidar_frame, 0.01, false);

    // create scan pose
    ScanPose SP(cloud_in_lidar_frame, stamp_current, T_WORLD_BASELINK_pert,
                T_BASELINK_LIDAR_, feature_extractor_);

    scan_poses.push_back(SP);
    gt_poses.push_back(T_WORLD_BASELINK);
    stamp_current = stamp_current + time_inc;
  }

  // create submaps (we will add them all to the same submap)
  // NOTE: we set T_WORLD_SUBMAP to identity so that T_WORLD_BASELINK =
  // T_SUBMAP_BASELINK to make things simpler
  Eigen::Matrix4d T_WORLD_SUBMAP = Eigen::Matrix4d::Identity();
  SubmapPtr submap = std::make_shared<Submap>(
      scan_poses.at(0).Stamp(), T_WORLD_SUBMAP, nullptr, extrinsics_);
  for (const ScanPose& sp : scan_poses) {
    submap->AddLidarMeasurement(sp.Cloud(), sp.T_REFFRAME_BASELINK(),
                                sp.Stamp(), 0);
    submap->AddLidarMeasurement(sp.LoamCloud().edges.strong.cloud,
                                sp.T_REFFRAME_BASELINK(), sp.Stamp(), 1);
    submap->AddLidarMeasurement(sp.LoamCloud().surfaces.strong.cloud,
                                sp.T_REFFRAME_BASELINK(), sp.Stamp(), 2);
    submap->AddLidarMeasurement(sp.LoamCloud().edges.weak.cloud,
                                sp.T_REFFRAME_BASELINK(), sp.Stamp(), 3);
    submap->AddLidarMeasurement(sp.LoamCloud().surfaces.weak.cloud,
                                sp.T_REFFRAME_BASELINK(), sp.Stamp(), 4);
  }
  std::vector<SubmapPtr> submaps{submap};

  // create global map
  std::shared_ptr<GlobalMap> global_map =
      std::make_shared<GlobalMap>(nullptr, extrinsics_);
  global_map->SetOnlineSubmaps(submaps);

  // load params
  GlobalMapRefinement::Params params;
  params.LoadJson(refinement_config_path_);
  params.scan_registration_type = "MULTISCAN";

  // create global map refinement
  GlobalMapRefinement refinement(global_map, params);
  refinement.RunSubmapRefinement();

  // submaps.at(0)->SaveLidarMapInWorldFrame(
  //     output_root_ + "multi_scan/cloud_refined.pcd", 5e6);
  // submaps.at(0)->SaveLidarMapInWorldFrame(
  //     output_root_ + "multi_scan/cloud_original.pcd", 5e6, true);

  //  Check poses
  int counter = 0;
  for (auto iter = submaps.at(0)->LidarKeyframesBegin();
       iter != submaps.at(0)->LidarKeyframesEnd(); iter++) {
    // iter->second.UpdatePose(graph);

    // save scans
    // iter->second.SaveCloud(output_root_ + "/multi_scan/");

    const Eigen::Matrix4d& T_WORLD_BASELINK_gt = gt_poses.at(counter);
    // make sure stamp is the same
    EXPECT_TRUE(iter->first == scan_poses.at(counter).Stamp().toNSec());

    // check pose
    EXPECT_TRUE(beam::ArePosesEqual(iter->second.T_REFFRAME_BASELINK(),
                                    T_WORLD_BASELINK_gt, 1.5, 0.05, true));

    counter++;
  }
}

TEST_F(GlobalMapRefinementTest, ScanToMap) {
  // create path
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> nodes;
  nodes.push_back(Eigen::Vector3d(0, 0, 0));
  nodes.push_back(Eigen::Vector3d(1, -0.5, 0));
  nodes.push_back(Eigen::Vector3d(3, 0.5, 0));
  nodes.push_back(Eigen::Vector3d(5, 0.7, 0));
  beam::SimplePathGenerator path(nodes);

  // create scan poses
  int num_scans = 15;
  ros::Time stamp_current = ros::Time(0);
  ros::Duration time_inc = ros::Duration(1);  // increment by 1 s
  std::vector<ScanPose> scan_poses;
  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>
      gt_poses;
  for (size_t i = 0; i < num_scans - 5; i++) {
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
    PointCloud cloud_in_lidar_frame;
    pcl::transformPointCloud(
        cloud_in_world_frame_, cloud_in_lidar_frame,
        beam::InvertTransform(T_WORLD_BASELINK * T_BASELINK_LIDAR_));
    beam::AddNoiseToCloud(cloud_in_lidar_frame, 0.01, false);

    // create scan pose
    ScanPose SP(cloud_in_lidar_frame, stamp_current, T_WORLD_BASELINK_pert,
                T_BASELINK_LIDAR_, feature_extractor_);

    scan_poses.push_back(SP);
    gt_poses.push_back(T_WORLD_BASELINK);
    stamp_current = stamp_current + time_inc;
  }

  // create submaps (we will add them all to the same submap)
  // NOTE: we set T_WORLD_SUBMAP to identity so that T_WORLD_BASELINK =
  // T_SUBMAP_BASELINK to make things simpler
  Eigen::Matrix4d T_WORLD_SUBMAP = Eigen::Matrix4d::Identity();
  SubmapPtr submap = std::make_shared<Submap>(
      scan_poses.at(0).Stamp(), T_WORLD_SUBMAP, nullptr, extrinsics_);
  for (const ScanPose& sp : scan_poses) {
    submap->AddLidarMeasurement(sp.Cloud(), sp.T_REFFRAME_BASELINK(),
                                sp.Stamp(), 0);
    submap->AddLidarMeasurement(sp.LoamCloud().edges.strong.cloud,
                                sp.T_REFFRAME_BASELINK(), sp.Stamp(), 1);
    submap->AddLidarMeasurement(sp.LoamCloud().surfaces.strong.cloud,
                                sp.T_REFFRAME_BASELINK(), sp.Stamp(), 2);
    submap->AddLidarMeasurement(sp.LoamCloud().edges.weak.cloud,
                                sp.T_REFFRAME_BASELINK(), sp.Stamp(), 3);
    submap->AddLidarMeasurement(sp.LoamCloud().surfaces.weak.cloud,
                                sp.T_REFFRAME_BASELINK(), sp.Stamp(), 4);
  }
  std::vector<SubmapPtr> submaps{submap};

  // create global map
  std::shared_ptr<GlobalMap> global_map =
      std::make_shared<GlobalMap>(nullptr, extrinsics_);
  global_map->SetOnlineSubmaps(submaps);

  // load params
  GlobalMapRefinement::Params params;
  params.LoadJson(refinement_config_path_);
  params.scan_registration_type = "SCANTOMAP";

  // create global map refinement
  GlobalMapRefinement refinement(global_map, params);
  refinement.RunSubmapRefinement();

  // submaps.at(0)->SaveLidarMapInWorldFrame(
  //     output_root_ + "scan_to_map/cloud_refined.pcd", 5e6);
  // submaps.at(0)->SaveLidarMapInWorldFrame(
  //     output_root_ + "scan_to_map/cloud_original.pcd", 5e6, true);

  //  Check poses
  int counter = 0;
  for (auto iter = submaps.at(0)->LidarKeyframesBegin();
       iter != submaps.at(0)->LidarKeyframesEnd(); iter++) {
    // iter->second.UpdatePose(graph);

    // save scans
    // iter->second.SaveCloud(output_root_ + "/scan_to_map/");

    const Eigen::Matrix4d& T_WORLD_BASELINK_gt = gt_poses.at(counter);
    // make sure stamp is the same
    EXPECT_TRUE(iter->first == scan_poses.at(counter).Stamp().toNSec());

    // check pose
    EXPECT_TRUE(beam::ArePosesEqual(iter->second.T_REFFRAME_BASELINK(),
                                    T_WORLD_BASELINK_gt, 1.5, 0.05, true));

    counter++;
  }
}

/*
TEST_F(GlobalMapRefinementTest, MultiScanRealData) {
  std::string globalmap_dir =
      "/home/nick/results/beam_slam/global_map_refinement/test/GlobalMapData/";
  std::string output_dir =
      "/home/nick/results/beam_slam/global_map_refinement/test/ResultsRefinedUnitTets/";

  // load params
  GlobalMapRefinement::Params params;
  params.LoadJson(refinement_config_path_);
  params.scan_registration_type = "MULTISCAN";

  // create global map refinement
  GlobalMapRefinement refinement(globalmap_dir, params);
  refinement.RunSubmapRefinement();
  refinement.SaveResults(output_dir);

}
*/

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
