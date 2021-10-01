#pragma once

#include <map>

#include <ros/time.h>

#include <beam_utils/pointclouds.h>
#include <bs_models/global_mapping/submap.h>

namespace bs_models {

namespace reloc {

using namespace global_mapping;

/**
 * @brief Reloc candidate search finds candidate relocs within submaps
 * and returns estimated relative poses
 */
class RelocCandidateSearchBase {
 public:
  /**
   * @brief constructor with an optional path to a json config
   * @param config path to json config file. If empty, it will use default
   * parameters
   */
  RelocCandidateSearchBase(const std::string& config = "")
      : config_path_(config) {}

  /**
   * @brief default destructor
   */
  ~RelocCandidateSearchBase() = default;

  /**
   * @brief Pure virtual function that takes in a vector of submaps, a query
   * pose and finds candidate relocs with an estimated relative pose. The
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
  virtual void FindRelocCandidates(
      const std::vector<SubmapPtr>& submaps,
      const Eigen::Matrix4d& T_WORLD_QUERY, std::vector<int>& matched_indices,
      std::vector<Eigen::Matrix4d, beam::AlignMat4d>& estimated_poses,
      size_t ignore_last_n_submaps = 0, bool use_initial_poses = false) = 0;

 protected:
  /**
   * @brief pure virtual method for loading a config json file.
   */
  virtual void LoadConfig() = 0;

  std::string config_path_;
};

}  // namespace reloc

}  // namespace bs_models
