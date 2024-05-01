#include <bs_models/global_mapping/global_map_batch_optimization.h>

#include <filesystem>

#include <fuse_core/transaction.h>

#include <beam_matching/Matchers.h>
#include <beam_matching/Scancontext.h>
#include <beam_utils/pointclouds.h>

#include <bs_common/conversions.h>
#include <bs_common/graph_access.h>
#include <bs_common/utils.h>
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

  // Get a map of candidate scans that are within the dist thresholds, by
  // reverse iterating over all scan poses starting from the query timestamp
  std::map<double, std::pair<uint64_t, int>>
      candidates; // dist -> (scan_stamp, submap_id)
  auto tmp = scan_stamp_to_submap_id_.find(timestamp_query_ns);
  auto it_start = std::reverse_iterator(std::next(tmp));
  for (auto it = it_start; it != scan_stamp_to_submap_id_.rend(); it++) {
    // Get candidate scan
    const auto candidate_stamp = it->first;
    const auto candidate_submap_id = it->second;
    const auto& candidate =
        submaps_.at(candidate_submap_id)->LidarKeyframes().at(candidate_stamp);
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

    candidates.emplace(dist_to_candidate,
                       std::make_pair(candidate_stamp, candidate_submap_id));
  }

  // iterate by smallest distance and store measurements
  std::vector<LoopClosureMeasurement> measurements;
  for (auto it = candidates.begin(); it != candidates.end(); it++) {
    const auto candidate_stamp = it->second.first;
    const auto candidate_submap_id = it->second.second;
    const auto& candidate =
        submaps_.at(candidate_submap_id)->LidarKeyframes().at(candidate_stamp);

    // check scan context
    PointCloudSC candidate_scan = AggregateScan(candidate_stamp);
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

    LoopClosureMeasurement measurement;
    measurement.T_Query_Candidate_Measured = T_Query_Candidate_Measured;
    measurement.covariance = cov;
    measurement.candidate_position = candidate.Position();
    measurement.candidate_orientation = candidate.Orientation();
    measurement.scan_context_dist = sc_result.first;
    measurements.push_back(measurement);

    if (params_.lc_max_per_query_scan > 0 &&
        measurements.size() >= params_.lc_max_per_query_scan) {
      break;
    }
  }

  measurements = RemoveOutlierMeasurements(measurements);
  for (const auto& m : measurements) {
    bs_constraints::Pose3DStampedTransaction transaction(query.Stamp());
    transaction.AddPoseConstraint(
        m.candidate_position, query.Position(), m.candidate_orientation,
        query.Orientation(),
        bs_common::TransformMatrixToVectorWithQuaternion(
            beam::InvertTransform(m.T_Query_Candidate_Measured)),
        m.covariance, "GlobalMapBatchOptimization::RunLoopClosureOnAllScans",
        lidar_frame_id_);

    graph_->update(*(transaction.GetTransaction()));
  }

  if (measurements.empty()) { return; }

  // optimize
  if (params_.update_graph_on_all_lcs) {
    BEAM_INFO("Optimizing the graph with {} loop closure constraints",
              measurements.size());
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
              measurements.size(), std::to_string(query.Stamp().toSec()));
  }
}

std::vector<GlobalMapBatchOptimization::LoopClosureMeasurement>
    GlobalMapBatchOptimization::RemoveOutlierMeasurements(
        const std::vector<GlobalMapBatchOptimization::LoopClosureMeasurement>&
            measurements) const {
  // we need at least a few measurements to be able to properly estimate
  // statistics
  if (measurements.size() < min_measurements_for_outlier_rejection_) {
    return measurements;
  }

  // calculate the metrics we care about and the sum at the same time
  OutlierRejectionMetrics metrics_sum;
  std::vector<OutlierRejectionMetrics> metrics;
  for (const LoopClosureMeasurement& m : measurements) {
    OutlierRejectionMetrics metric;
    Eigen::Matrix4d T_World_Candidate = bs_common::FusePoseToEigenTransform(
        m.candidate_position, m.candidate_orientation);
    Eigen::Matrix4d T_World_Query =
        T_World_Candidate * beam::InvertTransform(m.T_Query_Candidate_Measured);
    Eigen::Matrix3d R = T_World_Query.block(0, 0, 3, 3);
    Eigen::AngleAxisd aa(R);
    metric.t = T_World_Query.block(0, 3, 3, 1).norm();
    metric.r = aa.angle();
    metric.d = m.scan_context_dist;
    metric.e = bs_common::ShannonEntropyFromPoseCovariance(m.covariance);
    metrics.push_back(metric);

    metrics_sum.t += metric.t;
    metrics_sum.r += metric.r;
    metrics_sum.d += metric.d;
    metrics_sum.e += metric.e;
  }

  OutlierRejectionMetrics metrics_mean;
  metrics_mean.t = metrics_sum.t / metrics.size();
  metrics_mean.r = metrics_sum.r / metrics.size();
  metrics_mean.d = metrics_sum.d / metrics.size();
  metrics_mean.e = metrics_sum.e / metrics.size();

  OutlierRejectionMetrics metrics_diffsquared;
  for (const OutlierRejectionMetrics& m : metrics) {
    metrics_diffsquared.t += (m.t - metrics_mean.t) * (m.t - metrics_mean.t);
    metrics_diffsquared.r += (m.r - metrics_mean.r) * (m.r - metrics_mean.r);
    metrics_diffsquared.d += (m.d - metrics_mean.d) * (m.d - metrics_mean.d);
    metrics_diffsquared.e += (m.e - metrics_mean.e) * (m.e - metrics_mean.e);
  }

  OutlierRejectionMetrics metrics_stddev;
  metrics_stddev.t = std::sqrt(metrics_diffsquared.t / metrics.size());
  metrics_stddev.r = std::sqrt(metrics_diffsquared.r / metrics.size());
  metrics_stddev.d = std::sqrt(metrics_diffsquared.d / metrics.size());
  metrics_stddev.e = std::sqrt(metrics_diffsquared.e / metrics.size());

  // check save if metric is outside mean +/- 2 std. dev.
  std::vector<LoopClosureMeasurement> measurements_filtered;
  for (int i = 0; i < measurements.size(); i++) {
    const auto& metric = metrics.at(i);
    if (!CheckMetric(metric.t, metrics_mean.t, metrics_stddev.t,
                     "failed translation check")) {
      continue;
    } else if (!CheckMetric(metric.r, metrics_mean.r, metrics_stddev.r,
                            "failed rotation check")) {
      continue;
    } else if (metric.e > metrics_mean.e + 2 * metrics_stddev.e) {
      BEAM_WARN(
          "failed shannon entropy check, value: {}, expected range: <= {}]",
          metric.e, metrics_mean.e + 2 * metrics_stddev.e);
      continue;
    } else if (metric.d > metrics_mean.d + 2 * metrics_stddev.d) {
      BEAM_WARN(
          "failed scan context check. Distance: {} > mean + 2 std. dev = {}",
          metric.d, metrics_mean.d + 2 * metrics_stddev.d);
      continue;
    }
    measurements_filtered.push_back(measurements.at(i));
  }
  return measurements_filtered;
}

bool GlobalMapBatchOptimization::CheckMetric(double value, double mean,
                                             double std_dev,
                                             const std::string& error) const {
  if (value < mean - 2 * std_dev || value > mean + 2 * std_dev) {
    BEAM_WARN("{}, value: {}, expected range: [{}, {}]", error, value,
              mean - 2 * std_dev, mean + 2 * std_dev);
    return false;
  }
  return true;
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
}

} // namespace bs_models::global_mapping