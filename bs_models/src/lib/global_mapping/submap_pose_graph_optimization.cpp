#include <bs_models/global_mapping/submap_pose_graph_optimization.h>

#include <fuse_core/transaction.h>

#include <bs_models/reloc/reloc_candidate_search_base.h>
#include <bs_models/reloc/reloc_methods.h>
#include <bs_models/reloc/reloc_refinement_base.h>

namespace bs_models::global_mapping {

using namespace reloc;
using namespace beam_matching;

using PointCloudSC = pcl::PointCloud<pcl::PointXYZI>;
using PointCloudSCPtr = std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>;

SubmapPoseGraphOptimization::SubmapPoseGraphOptimization(
    const SubmapPoseGraphOptimization::Params& params,
    const std::string& output_path)
    : params_(params), output_path_(output_path) {}

bool SubmapPoseGraphOptimization::Run(std::vector<SubmapPtr> submaps) {
  // Setup
  ros::Time::init();
  std::shared_ptr<reloc::RelocCandidateSearchBase>
      loop_closure_candidate_search = reloc::RelocCandidateSearchBase::Create(
          params_.candidate_search_config);
  std::shared_ptr<reloc::RelocRefinementBase> loop_closure_refinement =
      reloc::RelocRefinementBase::Create(params_.refinement_config);

  size_t num_submaps = submaps.size();
  if (num_submaps <= pgo_skip_first_n_submaps_) {
    BEAM_ERROR("Global map size {} not large enough to run PGO, must have at "
               "least {} submaps",
               num_submaps, pgo_skip_first_n_submaps_);
  }

  std::string lc_results_path_refinement =
      beam::CombinePaths(output_path_, "refinement");
  std::string lc_results_path_candidate_search =
      beam::CombinePaths(output_path_, "candidate_search");
  std::filesystem::create_directory(lc_results_path_refinement);
  std::filesystem::create_directory(lc_results_path_candidate_search);

  BEAM_INFO("Running pose-graph optimization on submaps");
  std::shared_ptr<fuse_graphs::HashGraph> graph =
      fuse_graphs::HashGraph::make_shared();

  // add first pose prior
  {
    const SubmapPtr& first_submap = submaps.at(0);
    bs_constraints::Pose3DStampedTransaction prior_transaction(
        first_submap->Stamp());
    prior_transaction.AddPoseVariables(first_submap->Position(),
                                       first_submap->Orientation(),
                                       first_submap->Stamp());
    prior_transaction.AddPosePrior(
        first_submap->Position(), first_submap->Orientation(),
        pose_prior_noise_fixed_, "SubmapPoseGraphOptimization::Run");
    graph->update(*prior_transaction.GetTransaction());
  }

  // add all relative pose transactions
  fuse_core::Transaction::SharedPtr transaction =
      std::make_shared<fuse_core::Transaction>();
  for (int i = 1; i < num_submaps; i++) {
    const SubmapPtr& current_submap = submaps.at(i);
    bs_constraints::Pose3DStampedTransaction new_transaction(
        current_submap->Stamp());
    new_transaction.AddPoseVariables(current_submap->Position(),
                                     current_submap->Orientation(),
                                     current_submap->Stamp());

    // If first submap, continue
    if (i < 1) { continue; }

    // add relative constraint to prev
    const SubmapPtr& previous_submap = submaps.at(i - 1);
    Eigen::Matrix4d T_PREVIOUS_CURRENT =
        beam::InvertTransform(previous_submap->T_WORLD_SUBMAP()) *
        current_submap->T_WORLD_SUBMAP();
    new_transaction.AddPoseConstraint(
        previous_submap->Position(), current_submap->Position(),
        previous_submap->Orientation(), current_submap->Orientation(),
        bs_common::TransformMatrixToVectorWithQuaternion(T_PREVIOUS_CURRENT),
        params_.local_mapper_covariance, "SubmapPoseGraphOptimization::Run");
    graph->update(*new_transaction.GetTransaction());
  }

  // now iterate through all submaps, check if loop closures can be run, and if
  // so, update graph after each loop closure
  for (int query_index = pgo_skip_first_n_submaps_; query_index < num_submaps;
       query_index++) {
    std::vector<int> matched_indices;
    std::vector<Eigen::Matrix4d, beam::AlignMat4d> Ts_MATCH_QUERY;

    // ignore all submaps equal to or after the submap before query.
    // I.e. if query is 2 and we have 5 submaps, ignore 1, 2, 3, 4. Check 0.
    // (size = 5 - 2 + 1 = 4)
    int ignore_last_n_submaps = submaps.size() - query_index + 1;
    BEAM_INFO("Finding reloc candidates for query submap id: {}", query_index);
    loop_closure_candidate_search->FindRelocCandidates(
        submaps, submaps.at(query_index), matched_indices, Ts_MATCH_QUERY,
        ignore_last_n_submaps, lc_results_path_candidate_search);
    std::string candidates;
    for (const auto& id : matched_indices) {
      candidates += std::to_string(id) + " ";
    }

    if (matched_indices.size() == 0) {
      BEAM_INFO("No loop closure candidates for query index {}", query_index);
      continue;
    }

    BEAM_INFO(
        "Found {} loop closure candidates for query index {}. Candidates: {}",
        matched_indices.size(), query_index, candidates);

    auto transaction = std::make_shared<fuse_core::Transaction>();
    for (int i = 0; i < matched_indices.size(); i++) {
      if (matched_indices[i] >= query_index - 1) {
        BEAM_ERROR("Error in candidate search implementation, please fix!");
        continue;
      }

      const auto& matched_submap = submaps.at(matched_indices[i]);
      const auto& query_submap = submaps.at(query_index);
      RelocRefinementResults results = loop_closure_refinement->RunRefinement(
          matched_submap, query_submap, Ts_MATCH_QUERY[i],
          lc_results_path_refinement);

      if (!results.successful) { continue; }

      bs_constraints::Pose3DStampedTransaction new_transaction(
          query_submap->Stamp());
      new_transaction.AddPoseConstraint(
          matched_submap->Position(), query_submap->Position(),
          matched_submap->Orientation(), query_submap->Orientation(),
          bs_common::TransformMatrixToVectorWithQuaternion(
              results.T_MATCH_QUERY),
          params_.loop_closure_covariance, "SubmapPoseGraphOptimization::Run");
      transaction->merge(*(new_transaction.GetTransaction()));
    }

    graph->update(*transaction);
    graph->optimize();
    UpdateSubmapPosesFromGraph(submaps, graph);
  }

  return true;
}

} // namespace bs_models::global_mapping