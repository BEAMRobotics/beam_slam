#pragma once

#include <map>

#include <ros/time.h>

#include <beam_cv/ImageDatabase.h>
#include <bs_models/reloc/reloc_candidate_search_base.h>

namespace bs_models::reloc {

/**
 * @brief This class implements a reloc candidate search class. To
 * look for candidate relocs, this class queries an image database and returns
 * the matching submaps
 */
class RelocCandidateSearchVisual : public RelocCandidateSearchBase {
public:
  /**
   * @brief constructor that takes in a poitner to the image database to use
   * @param image_database
   */
  RelocCandidateSearchVisual(
      const std::shared_ptr<beam_cv::ImageDatabase>& image_database);

  /**
   * @brief Overrides the virtual function that takes in a vector of submaps, a
   * query pose and finds candidate relocs with an estimated relative pose. The
   * candidates should be ordered based on the most likely candidate.
   * @param submaps vector of pointers to submaps
   * @param T_WORLD_QUERY we look for submaps that contains this pose (note
   * query pose is the pose of the baselink)
   * @param matched_indices reference to vector of indices which represent the
   * candidate reloc submap indices
   * @param estimated_poses reference to vector of transforms from query pose
   * to matched submap (T_SUBMAPCANDIDATE_QUERY)
   * @param ignore_last_n_submaps how many of the final submaps should be
   * ignored. This is useful when using this for loop closure where we don't
   * want to look in the last n submaps
   * @param use_initial_poses if set to true, we will use the initial pose in
   * the submaps. This is useful when running reloc on submaps that are being
   * optimized, but the initial query pose estimate is in the original world
   * frame
   */
  void FindRelocCandidates(
      const std::vector<global_mapping::SubmapPtr>& submaps,
      const Eigen::Matrix4d& T_WORLD_QUERY,
      const std::vector<cv::Mat>& query_images,
      std::vector<int>& matched_indices,
      std::vector<Eigen::Matrix4d, beam::AlignMat4d>& estimated_poses,
      size_t ignore_last_n_submaps = 0,
      bool use_initial_poses = false) override;

private:
  /**
   * @brief Method for loading a config json file.
   */
  void LoadConfig() override;

  std::shared_ptr<beam_cv::ImageDatabase> image_database_;
};

} // namespace bs_models::reloc
