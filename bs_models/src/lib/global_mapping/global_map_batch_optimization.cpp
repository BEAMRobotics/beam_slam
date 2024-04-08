#include <bs_models/global_mapping/global_map_batch_optimization.h>

#include <filesystem>

#include <fuse_core/transaction.h>

#include <beam_matching/Matchers.h>
#include <beam_matching/Scancontext.h>
#include <beam_utils/pointclouds.h>

#include <bs_common/graph_access.h>
#include <bs_common/visualization.h>
#include <bs_constraints/relative_pose/relative_pose_3d_stamped_with_extrinsics_constraint.h>
#include <bs_models/graph_visualization/helpers.h>
#include <bs_models/lidar/scan_pose.h>
#include <bs_models/reloc/reloc_candidate_search_base.h>
#include <bs_models/reloc/reloc_methods.h>
#include <bs_models/reloc/reloc_refinement_base.h>
#include <bs_models/scan_registration/multi_scan_registration.h>
#include <bs_models/scan_registration/scan_to_map_registration.h>

namespace bs_models::global_mapping {

using namespace reloc;
using namespace beam_matching;

namespace sr = bs_models::scan_registration;

using PointCloudSC = pcl::PointCloud<pcl::PointXYZI>;
using PointCloudSCPtr = std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>;

GlobalMapBatchOptimization::GlobalMapBatchOptimization(
    const Params& params, const std::string& output_path)
    : params_(params), output_path_(output_path) {
  // Setup matchers
  const auto& m_conf = params_.matcher_config;
  auto matcher_type = GetTypeFromConfig(m_conf);
  if (matcher_type == MatcherType::LOAM) {
    std::string ceres_config =
        bs_common::GetAbsoluteConfigPathFromJson(m_conf, "ceres_config");
    matcher_loam_ =
        std::make_unique<LoamMatcher>(LoamParams(m_conf, ceres_config));
  } else if (matcher_type == MatcherType::ICP) {
    matcher_ = std::make_unique<IcpMatcher>(IcpMatcher::Params(m_conf));
  } else if (matcher_type == MatcherType::GICP) {
    matcher_ = std::make_unique<GicpMatcher>(GicpMatcher::Params(m_conf));
  } else if (matcher_type == MatcherType::NDT) {
    matcher_ = std::make_unique<NdtMatcher>(NdtMatcher::Params(m_conf));
  } else {
    BEAM_ERROR("Invalid matcher type");
    throw std::invalid_argument{"invalid json"};
  }

  // setup graph
  graph_ = fuse_graphs::HashGraph::make_shared();
}

bool GlobalMapBatchOptimization::Run(std::vector<SubmapPtr> submaps) {
  // get lidar frame id
  lidar_frame_id_ = submaps.at(0)->Extrinsics()->GetLidarFrameId();

  ConvertScanPosesToWorld(submaps);

  // get mapping from all scan stamps, to their respective submap ids
  scan_stamp_to_submap_id_.clear();
  for (int i = 0; i < submaps_.size(); i++) {
    for (auto scan_iter = submaps_.at(i)->LidarKeyframesBegin();
         scan_iter != submaps_.at(i)->LidarKeyframesEnd(); scan_iter++) {
      scan_stamp_to_submap_id_.emplace(scan_iter->first, i);
    }
  }

  // setup scan registration
  std::unique_ptr<sr::ScanRegistrationBase> scan_registration =
      sr::ScanRegistrationBase::Create(params_.scan_registration_config,
                                       params_.matcher_config, "", 1e-9);

  // clear lidar map
  scan_registration->GetMapMutable().Clear();

  BEAM_INFO("Running batch optimization");
  for (int i = 0; i < submaps_.size(); i++) {
    BEAM_INFO("Working on submap No. {}", i);
    const auto& submap = submaps_.at(i);
    for (auto scan_iter = submap->LidarKeyframesBegin();
         scan_iter != submap->LidarKeyframesEnd(); scan_iter++) {
      auto transaction = scan_registration->RegisterNewScan(scan_iter->second)
                             .GetTransaction();
      if (transaction) { graph_->update(*transaction); }

      if (params_.update_graph_on_all_scans) {
        graph_->optimize();
        UpdateSubmapScanPosesFromGraph(submaps_, graph_, scan_iter->first,
                                       true);
        UpdateSubmapPosesFromGraph(submaps_, graph_);
      }

      RunLoopClosureOnAllScans(scan_iter->second);
    }
  }

  // visualize before
  std::filesystem::path p(output_path_);
  std::filesystem::path init_poses_path =
      p / std::filesystem::path("graph_poses_before_opt.pcd");
  std::filesystem::path init_constraints_path =
      p / std::filesystem::path("graph_constraints_before_opt.pcd");
  std::filesystem::path init_constraints_path2 =
      p / std::filesystem::path("graph_constraints_before_opt_no_poses.pcd");
  std::filesystem::path init_constraints_lc_path =
      p / std::filesystem::path("graph_constraints_lc_before_opt.pcd");
  auto cloud = bs_common::GetGraphPosesAsCloud(*graph_);
  beam::SavePointCloud(init_poses_path.string(), cloud);
  cloud = bs_models::graph_visualization::
      GetGraphRelativePoseWithExtrinsicsConstraintsAsCloud(*graph_);
  beam::SavePointCloud(init_constraints_path.string(), cloud);
  cloud = bs_models::graph_visualization::
      GetGraphRelativePoseWithExtrinsicsConstraintsAsCloud(*graph_, "", false);
  beam::SavePointCloud(init_constraints_path2.string(), cloud);
  cloud = bs_common::GetGraphRelativePoseConstraintsAsCloud(*graph_, "");
  beam::SavePointCloud(init_constraints_lc_path.string(), cloud);

  // optimize
  BEAM_INFO("Running final graph optimization");
  graph_->optimize();

  // visualize after
  std::filesystem::path opt_poses_path =
      p / std::filesystem::path("graph_poses_after_opt.pcd");
  std::filesystem::path opt_constraints_path =
      p / std::filesystem::path("graph_constraints_after_opt.pcd");
  std::filesystem::path opt_constraints_path2 =
      p / std::filesystem::path("graph_constraints_after_opt_no_poses.pcd");
  std::filesystem::path opt_constraints_lc_path =
      p / std::filesystem::path("graph_constraints_lc_after_opt.pcd");
  cloud = bs_common::GetGraphPosesAsCloud(*graph_);
  beam::SavePointCloud(opt_poses_path.string(), cloud);
  cloud = bs_models::graph_visualization::
      GetGraphRelativePoseWithExtrinsicsConstraintsAsCloud(*graph_);
  beam::SavePointCloud(opt_constraints_path.string(), cloud);
  cloud = bs_models::graph_visualization::
      GetGraphRelativePoseWithExtrinsicsConstraintsAsCloud(*graph_, "", false);
  beam::SavePointCloud(opt_constraints_path2.string(), cloud);
  cloud = bs_common::GetGraphRelativePoseConstraintsAsCloud(*graph_, "");
  beam::SavePointCloud(opt_constraints_lc_path.string(), cloud);

  static bool verbose = false;
  UpdateSubmapScanPosesFromGraph(submaps_, graph_, 0, true, verbose);
  UpdateSubmapPosesFromGraph(submaps_, graph_, verbose);
  UpdateInputSubmaps(submaps);
  return true;
}

void GlobalMapBatchOptimization::RunLoopClosureOnAllScans(ScanPose& query) {
  if (params_.lc_max_per_query_scan == 0) { return; }

  // Aggregate around scan for helping scan context
  uint64_t timestamp_query_ns = query.Stamp().toNSec();
  PointCloudSC query_scan_agg = AggregateScan(timestamp_query_ns);
  SCManager sc_manager;
  auto sc_query = sc_manager.makeScancontext(query_scan_agg);

  double trajectory_length = 0;
  Eigen::Matrix4d T_World_BaselinkQuery = query.T_REFFRAME_BASELINK();
  Eigen::Matrix4d T_World_Baselink_Last = T_World_BaselinkQuery;
  int new_measurements_count = 0;

  // get start iterator from scan_stamp_to_submap_id_
  auto tmp = scan_stamp_to_submap_id_.find(timestamp_query_ns);
  auto it_start = std::reverse_iterator(std::next(tmp));
  for (auto it = it_start; it != scan_stamp_to_submap_id_.rend(); it++) {
    // Get candidate scan
    const auto& candidate =
        submaps_.at(it->second)->LidarKeyframes().at(it->first);
    Eigen::Matrix4d T_World_BaselinkCandidate = candidate.T_REFFRAME_BASELINK();

    // check distance from scan, skip if lower than thresh
    Eigen::Matrix4d T_BaselinkLast_BaselinkCandidate =
        beam::InvertTransform(T_World_Baselink_Last) *
        T_World_BaselinkCandidate;
    T_World_Baselink_Last = T_World_BaselinkCandidate;
    double dist_from_last =
        T_BaselinkLast_BaselinkCandidate.block(0, 3, 3, 1).norm();
    trajectory_length += dist_from_last;
    if (trajectory_length < params_.lc_min_traj_dist_m) { continue; }

    // check min distance threshold for LC
    Eigen::Matrix4d T_BaselinkQuery_BaselinkCandidate =
        beam::InvertTransform(T_World_BaselinkQuery) *
        T_World_BaselinkCandidate;
    double dist_to_candidate =
        T_BaselinkQuery_BaselinkCandidate.block(0, 3, 3, 1).norm();
    if (dist_to_candidate > params_.lc_dist_thresh_m) { continue; }

    // check scan context
    PointCloudSC candidate_scan = AggregateScan(it->first);
    auto sc_candidate = sc_manager.makeScancontext(candidate_scan);
    std::pair<double, int> sc_result =
        sc_manager.distanceBtnScanContext(sc_query, sc_candidate);
    if (sc_result.first > params_.lc_scan_context_dist_thres) { continue; }

    // register scans
    Eigen::Matrix4d T_Query_Candidate_Init =
        beam::InvertTransform(query.T_REFFRAME_LIDAR()) *
        candidate.T_REFFRAME_LIDAR();
    Eigen::Matrix4d T_Query_Candidate_Measured;
    Eigen::Matrix<double, 6, 6> cov =
        params_.lc_cov_multiplier * Eigen::Matrix<double, 6, 6>::Identity();
    if (matcher_loam_) {
      auto query_cloud = std::make_shared<LoamPointCloud>(query.LoamCloud());
      auto candidate_in_query_est = std::make_shared<LoamPointCloud>(
          candidate.LoamCloud(), T_Query_Candidate_Init);
      matcher_loam_->SetRef(query_cloud);
      matcher_loam_->SetTarget(candidate_in_query_est);
      bool match_success = matcher_loam_->Match();
      T_Query_Candidate_Measured =
          matcher_loam_->ApplyResult(T_Query_Candidate_Init);
      cov = params_.lc_cov_multiplier * matcher_loam_->GetCovariance();
    } else {
      auto query_cloud = std::make_shared<PointCloud>(query.Cloud());
      auto candidate_cloud = candidate.Cloud();
      auto candidate_in_query_est = std::make_shared<PointCloud>();
      pcl::transformPointCloud(candidate_cloud, *candidate_in_query_est,
                               Eigen::Affine3d(T_Query_Candidate_Init));
      matcher_->SetRef(query_cloud);
      matcher_->SetTarget(candidate_in_query_est);
      if (!matcher_->Match()) {
        BEAM_ERROR("Match not successful, skipping loop closure measurement");
        continue;
      }
      T_Query_Candidate_Measured =
          matcher_->ApplyResult(T_Query_Candidate_Init);
    }

    // add transaction to the graph
    new_measurements_count++;
    bs_constraints::Pose3DStampedTransaction transaction(query.Stamp());
    transaction.AddPoseConstraint(
        candidate.Position(), query.Position(), candidate.Orientation(),
        query.Orientation(),
        bs_common::TransformMatrixToVectorWithQuaternion(
            beam::InvertTransform(T_Query_Candidate_Measured)),
        cov, "GlobalMapBatchOptimization::RunLoopClosureOnAllScans",
        lidar_frame_id_);

    graph_->update(*(transaction.GetTransaction()));
    if (params_.lc_max_per_query_scan > 0 &&
        new_measurements_count >= params_.lc_max_per_query_scan) {
      break;
    }
  }

  if (new_measurements_count == 0) { return; }

  // optimize
  if (params_.update_graph_on_all_lcs) {
    BEAM_INFO("Optimizing the graph with {} loop closure constraints",
              new_measurements_count);
    graph_->optimize();
    // update all scans
    BEAM_INFO("Updating submap poses");
    UpdateSubmapScanPosesFromGraph(submaps_, graph_, query.Stamp().toNSec(),
                                   true);
    UpdateSubmapPosesFromGraph(submaps_, graph_);
    BEAM_INFO("Done updating submap poses");
  } else {
    BEAM_INFO("Added {} loop closure constraints to the graph for query scan "
              "with time: {} s",
              new_measurements_count, std::to_string(query.Stamp().toSec()));
  }
}

PointCloudSC
    GlobalMapBatchOptimization::AggregateScan(uint64_t scan_time_ns) const {
  int submap_id = scan_stamp_to_submap_id_.at(scan_time_ns);
  const auto& scan_pose =
      submaps_.at(submap_id)->LidarKeyframes().at(scan_time_ns);
  Eigen::Matrix4d T_World_ScanCenter =
      submaps_.at(submap_id)->T_WORLD_SUBMAP() * scan_pose.T_REFFRAME_LIDAR();

  PointCloud cloud;
  pcl::copyPointCloud(scan_pose.Cloud(), cloud);

  // move backwards from scan
  auto tmp = scan_stamp_to_submap_id_.find(scan_time_ns);
  auto it_back = std::reverse_iterator(std::next(tmp));
  int count = 0;
  for (auto it = it_back; it != scan_stamp_to_submap_id_.rend(); it++) {
    const auto& submap = submaps_.at(it->second);
    const auto& scan_pose_to_agg = submap->LidarKeyframes().at(it->first);

    auto T_World_ScanToAgg =
        submap->T_WORLD_SUBMAP() * scan_pose_to_agg.T_REFFRAME_LIDAR();
    auto T_ScanCenter_ScanToAgg =
        beam::InvertTransform(T_World_ScanCenter) * T_World_ScanToAgg;
    PointCloud cloud_new;
    pcl::transformPointCloud(scan_pose_to_agg.Cloud(), cloud_new,
                             Eigen::Affine3d(T_ScanCenter_ScanToAgg));
    cloud += cloud_new;
    count++;
    if (count >= scans_to_aggregate_) { break; }
  }

  // move forward
  auto it_forw = scan_stamp_to_submap_id_.find(scan_time_ns);
  count = 0;
  for (auto it = it_forw; it != scan_stamp_to_submap_id_.end(); it++) {
    const auto& submap = submaps_.at(it->second);
    const auto& scan_pose_to_agg = submap->LidarKeyframes().at(it->first);

    auto T_World_ScanToAgg =
        submap->T_WORLD_SUBMAP() * scan_pose_to_agg.T_REFFRAME_LIDAR();
    auto T_ScanCenter_ScanToAgg =
        beam::InvertTransform(T_World_ScanCenter) * T_World_ScanToAgg;
    PointCloud cloud_new;
    pcl::transformPointCloud(scan_pose_to_agg.Cloud(), cloud_new,
                             Eigen::Affine3d(T_ScanCenter_ScanToAgg));
    cloud += cloud_new;
    count++;
    if (count >= scans_to_aggregate_) { break; }
  }

  beam_filtering::VoxelDownsample voxel_filter(scan_context_voxel_filter_);
  voxel_filter.SetInputCloud(std::make_shared<PointCloud>(cloud));
  voxel_filter.Filter();
  PointCloud pc_downsampled = voxel_filter.GetFilteredCloud();

  PointCloudSC cloud_out;
  pcl::copyPointCloud(pc_downsampled, cloud_out);
  return cloud_out;
}

void GlobalMapBatchOptimization::ConvertScanPosesToWorld(
    std::vector<SubmapPtr> submaps) {
  for (const auto& submap : submaps) {
    Submap submap_converted = *submap;
    for (auto& [_, scan_pose] : submap_converted.LidarKeyframesMutable()) {
      Eigen::Matrix4d T_World_Baselink =
          submap_converted.T_WORLD_SUBMAP() * scan_pose.T_REFFRAME_BASELINK();
      scan_pose.UpdatePose(T_World_Baselink);
    }
    submaps_.emplace_back(std::make_shared<Submap>(submap_converted));
  }
}

void GlobalMapBatchOptimization::UpdateInputSubmaps(
    std::vector<SubmapPtr> submaps) {
  for (int submap_it = 0; submap_it < submaps.size(); submap_it++) {
    Eigen::Matrix4d T_World_Submap = submaps_.at(submap_it)->T_WORLD_SUBMAP();
    submaps.at(submap_it)->UpdatePose(T_World_Submap);
    for (auto& [timestamp, scan_pose] :
         submaps.at(submap_it)->LidarKeyframesMutable()) {
      const auto& scan_pose_in_world =
          submaps_.at(submap_it)->LidarKeyframes().at(timestamp);
      Eigen::Matrix4d T_Submap_Baselink =
          beam::InvertTransform(T_World_Submap) *
          scan_pose_in_world.T_REFFRAME_BASELINK();
      scan_pose.UpdatePose(T_Submap_Baselink);
    }
  }
  // submaps = submaps_;
  // for (int submap_it = 0; submap_it < submaps.size(); submap_it++) {
  //   Eigen::Matrix4d T_World_Submap =
  //   submaps_.at(submap_it)->T_WORLD_SUBMAP(); for (auto& [timestamp,
  //   scan_pose] :
  //        submaps.at(submap_it)->LidarKeyframesMutable()) {
  //     const auto& scan_pose_in_world =
  //         submaps.at(submap_it)->LidarKeyframes().at(timestamp);
  //     Eigen::Matrix4d T_Submap_Baselink =
  //         beam::InvertTransform(T_World_Submap) *
  //         scan_pose_in_world.T_REFFRAME_BASELINK();
  //     scan_pose.UpdatePose(T_Submap_Baselink);
  //   }
  // }
}

} // namespace bs_models::global_mapping