#include <bs_models/reloc/reloc_candidate_search_visual.h>

#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>

#include <beam_utils/log.h>
#include <beam_utils/math.h>
#include <beam_utils/se3.h>

#include <bs_common/utils.h>

namespace bs_models { namespace reloc {

RelocCandidateSearchVisual::RelocCandidateSearchVisual(
    const std::shared_ptr<beam_cv::ImageDatabase>& image_database)
    : image_database_(image_database) {}

void RelocCandidateSearchVisual::FindRelocCandidates(
    const std::vector<SubmapPtr>& submaps, const Eigen::Matrix4d& T_WORLD_QUERY,
    const std::vector<cv::Mat>& query_images, std::vector<int>& matched_indices,
    std::vector<Eigen::Matrix4d, beam::AlignMat4d>& estimated_poses,
    size_t ignore_last_n_submaps, bool use_initial_poses) {
  if (submaps.size() <= ignore_last_n_submaps) { return; }

  // get all matches from the database for each query image passed
  std::vector<DBoW3::Result> all_database_results;
  auto query_database = [&](const auto& query_image) {
    auto cur_database_results = image_database_->QueryDatabase(query_image, 10);
    all_database_results.insert(all_database_results.end(),
                                cur_database_results.begin(),
                                cur_database_results.end());
  };
  std::for_each(query_images.begin(), query_images.end(), query_database);

  // for each result find its associated submap and its total score
  std::unordered_map<int, double> submap_score_map;
  for (const auto& res : all_database_results) {
    const auto stamp = image_database_->GetImageTimestamp(res.Id);
    if (!stamp.has_value()) { continue; }
    // find which submap this stamp is in, and increment said submaps score
    for (size_t i = 0; i < submaps.size() - ignore_last_n_submaps; i++) {
      if (submaps[i]->InSubmap(stamp.value())) {
        submap_score_map[i] += res.Score;
        break;
      }
    }
  }

  // sort candidates in decreasing order of score
  std::vector<std::pair<int, double>> candidate_submap_scores;
  std::transform(submap_score_map.begin(), submap_score_map.end(),
                 std::back_inserter(candidate_submap_scores),
                 [&](const auto& p) { return p; });
  std::sort(candidate_submap_scores.begin(), candidate_submap_scores.end(),
            [](auto& left, auto& right) { return left.second > right.second; });

  // get candidate submap poses
  auto get_candidate_submap_pose = [&](const auto& pair) {
    const auto index = pair.first;
    Eigen::Matrix4d T_WORLD_SUBMAPCANDIDATE;
    if (use_initial_poses) {
      T_WORLD_SUBMAPCANDIDATE = submaps.at(index)->T_WORLD_SUBMAP();
    } else {
      T_WORLD_SUBMAPCANDIDATE = submaps.at(index)->T_WORLD_SUBMAP_INIT();
    }
    Eigen::Matrix4d T_SUBMAPCANDIDATE_QUERY =
        beam::InvertTransform(T_WORLD_SUBMAPCANDIDATE) * T_WORLD_QUERY;
    return std::make_pair(index, T_SUBMAPCANDIDATE_QUERY);
  };
  std::vector<std::pair<int, Eigen::Matrix4d>> candidate_submap_poses;
  std::for_each(candidate_submap_scores.begin(), candidate_submap_scores.end(),
                get_candidate_submap_pose);

  // convert to output vectors
  matched_indices.clear();
  estimated_poses.clear();
  std::for_each(candidate_submap_poses.begin(), candidate_submap_poses.end(),
                [&](const auto& pair) {
                  matched_indices.push_back(pair.first);
                  estimated_poses.push_back(pair.second);
                });
}

void RelocCandidateSearchVisual::LoadConfig() {
  return;
}

}} // namespace bs_models::reloc
