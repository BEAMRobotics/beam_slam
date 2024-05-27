#include <bs_models/global_mapping/submap_refinement.h>

#include <filesystem>

#include <fuse_core/transaction.h>
#include <fuse_graphs/hash_graph.h>

// #include <beam_matching/Matchers.h>

#include <bs_constraints/relative_pose/relative_pose_3d_stamped_with_extrinsics_constraint.h>
#include <bs_models/scan_registration/multi_scan_registration.h>
#include <bs_models/scan_registration/scan_to_map_registration.h>

namespace bs_models::global_mapping {

using namespace beam_matching;

namespace sr = bs_models::scan_registration;

SubmapRefinement::SubmapRefinement(const SubmapRefinement::Params& params,
                                   const std::string& output_path)
    : params_(params), output_path_(output_path) {}

bool SubmapRefinement::Run(std::vector<SubmapPtr> submaps) {
  for (uint16_t i = 0; i < submaps.size(); i++) {
    BEAM_INFO("Refining submap No. {}/{}", i+1, submaps.size());
    if (!RefineSubmap(submaps.at(i))) {
      BEAM_ERROR("Submap refinement failed, exiting.");
      return false;
    }
  }
  return true;
}

bool SubmapRefinement::RefineSubmap(SubmapPtr& submap) {
  if (!output_path_.empty() && !std::filesystem::exists(output_path_)) {
    BEAM_ERROR("Invalid output path for submap refinement: {}", output_path_);
    throw std::runtime_error{"invalid path"};
  }
  std::string dir = "submap_" + std::to_string(submap->Stamp().toSec());
  std::string submap_output = output_path_.empty()
                                  ? output_path_
                                  : beam::CombinePaths(output_path_, dir);
  std::filesystem::create_directory(submap_output);

  // Create optimization graph
  std::shared_ptr<fuse_graphs::HashGraph> graph =
      fuse_graphs::HashGraph::make_shared();
  std::unique_ptr<sr::ScanRegistrationBase> scan_registration =
      sr::ScanRegistrationBase::Create(params_.scan_registration_config,
                                       params_.matcher_config, "", 1e-9);

  // clear lidar map
  scan_registration->GetMapMutable().Clear();

  // iterate through stored scan poses and add run scan registration. We store
  // the transactions and covariances to later add to the graph at the same time
  BEAM_INFO("Registering scans");
  std::map<ros::Time, fuse_core::Transaction::SharedPtr> reg_transactions;
  std::map<ros::Time, Eigen::Matrix<double, 6, 6>> reg_covariances;
  for (auto scan_iter = submap->LidarKeyframesBegin();
       scan_iter != submap->LidarKeyframesEnd(); scan_iter++) {
    const bs_models::ScanPose& sp = scan_iter->second;
    auto reg_transaction =
        scan_registration->RegisterNewScan(sp).GetTransaction();
    if (reg_transaction) {
      reg_transactions.emplace(sp.Stamp(), reg_transaction);

      // get covariance
      auto range = reg_transaction->addedConstraints();
      for (auto it = range.begin(); it != range.end(); it++) {
        if (it->type() ==
            "bs_constraints::RelativePose3DStampedWithExtrinsicsConstraint") {
          auto c =
              dynamic_cast<const bs_constraints::
                               RelativePose3DStampedWithExtrinsicsConstraint&>(
                  *it);
          reg_covariances.emplace(sp.Stamp(), c.covariance());
          break;
        }
      }
    }
  }

  // get average covariance diagonal to set prior covariance
  double cov_diag_sum = 0;
  for (const auto& [stamp, cov] : reg_covariances) {
    cov_diag_sum += cov.trace() / 6;
  }
  double average_cov_diag = cov_diag_sum / reg_covariances.size();

  // add priors
  for (auto scan_iter = submap->LidarKeyframesBegin();
       scan_iter != submap->LidarKeyframesEnd(); scan_iter++) {
    const bs_models::ScanPose& sp = scan_iter->second;
    bs_constraints::Pose3DStampedTransaction prior_transaction(sp.Stamp());
    prior_transaction.AddPoseVariables(sp.Position(), sp.Orientation(),
                                       sp.Stamp());
    prior_transaction.AddPosePrior(sp.Position(), sp.Orientation(),
                                   prior_cov_multiplyer_ * average_cov_diag,
                                   "GlobalMapRefinement::RefineSubmap");
    if (prior_transaction.GetTransaction()) {
      graph->update(*(prior_transaction.GetTransaction()));
    }
  }

  // add relative pose transactions
  for (const auto& [stamp, transaction] : reg_transactions) {
    graph->update(*transaction);
  }

  // Optimize graph and update data
  BEAM_INFO("Optimizing graph");
  graph->optimize();

  PointCloud submap_init;
  PointCloud submap_refined;
  BEAM_INFO("updating scan poses");
  for (auto scan_iter = submap->LidarKeyframesBegin();
       scan_iter != submap->LidarKeyframesEnd(); scan_iter++) {
    auto& sp = scan_iter->second;
    Eigen::Matrix4d T_W_B_init = sp.T_REFFRAME_BASELINK();
    sp.UpdatePose(graph);
    Eigen::Matrix4d T_W_B_after = sp.T_REFFRAME_BASELINK();
    RegistrationResult result(T_W_B_init, T_W_B_after);
    results_.emplace(sp.Stamp(), result);
    if (!submap_output.empty()) {
      auto scan_in_lidar = sp.Cloud();
      PointCloud scan_in_world_init;
      PointCloud scan_in_world_after;
      pcl::transformPointCloud(
          scan_in_lidar, scan_in_world_init,
          Eigen::Affine3d(T_W_B_init * sp.T_BASELINK_LIDAR()));
      pcl::transformPointCloud(
          scan_in_lidar, scan_in_world_after,
          Eigen::Affine3d(T_W_B_after * sp.T_BASELINK_LIDAR()));
      submap_init += scan_in_world_init;
      submap_refined += scan_in_world_after;
    }
  }

  if (!submap_output.empty()) {
    Eigen::Vector3d t_world_BaselinkStart =
        submap->LidarKeyframes().begin()->second.T_REFFRAME_BASELINK().block(
            0, 3, 3, 1);
    Eigen::Vector3d t_world_BaselinkEnd =
        submap->LidarKeyframes().rbegin()->second.T_REFFRAME_BASELINK().block(
            0, 3, 3, 1);

    std::string init_path =
        beam::CombinePaths(submap_output, "submap_initial.pcd");
    std::string ref_path =
        beam::CombinePaths(submap_output, "submap_refined.pcd");

    SaveViewableSubmap(init_path, submap_init, 255, 0, 0, t_world_BaselinkStart,
                       t_world_BaselinkEnd);
    SaveViewableSubmap(ref_path, submap_refined, 0, 255, 0,
                       t_world_BaselinkStart, t_world_BaselinkEnd);
  }

  return true;
}

} // namespace bs_models::global_mapping