#include <bs_models/global_mapping/submap_alignment.h>

#include <filesystem>

#include <beam_matching/Matchers.h>

namespace bs_models::global_mapping {

using namespace beam_matching;

SubmapAlignment::SubmapAlignment(const SubmapAlignment::Params& params,
                                 const std::string& output_path)
    : params_(params), output_path_(output_path) {}

bool SubmapAlignment::Run(std::vector<SubmapPtr> submaps) {
  if (submaps.size() < 2) {
    BEAM_WARN(
        "Not enough submaps to run submap alignment, at least two are needed");
    return true;
  }

  for (uint16_t i = 1; i < submaps.size(); i++) {
    BEAM_INFO("Aligning submap No. {}/{}", i+1, submaps.size());
    if (!AlignSubmaps(submaps.at(i - 1), submaps.at(i))) {
      BEAM_ERROR("Submap alignment failed, exiting.");
      return false;
    }
  }
  return true;
}

bool SubmapAlignment::AlignSubmaps(const SubmapPtr& submap_ref,
                                   SubmapPtr& submap_tgt) {
  if (!output_path_.empty() && !std::filesystem::exists(output_path_)) {
    BEAM_ERROR("Invalid output path for submap alignment: {}", output_path_);
    throw std::runtime_error{"invalid path"};
  }

  // Setup matchers
  std::unique_ptr<Matcher<PointCloudPtr>> matcher;
  std::unique_ptr<Matcher<LoamPointCloudPtr>> matcher_loam;
  const auto& m_conf = params_.matcher_config;
  auto matcher_type = GetTypeFromConfig(m_conf);
  if (matcher_type == MatcherType::LOAM) {
    std::string ceres_config =
        bs_common::GetAbsoluteConfigPathFromJson(m_conf, "ceres_config");
    matcher_loam =
        std::make_unique<LoamMatcher>(LoamParams(m_conf, ceres_config));
  } else if (matcher_type == MatcherType::ICP) {
    matcher = std::make_unique<IcpMatcher>(IcpMatcher::Params(m_conf));
  } else if (matcher_type == MatcherType::GICP) {
    matcher = std::make_unique<GicpMatcher>(GicpMatcher::Params(m_conf));
  } else if (matcher_type == MatcherType::NDT) {
    matcher = std::make_unique<NdtMatcher>(NdtMatcher::Params(m_conf));
  } else {
    BEAM_ERROR("Invalid matcher type");
    throw std::invalid_argument{"invalid json"};
  }

  std::string dir = "submap_" + std::to_string(submap_tgt->Stamp().toSec());
  std::string submap_output = output_path_.empty()
                                  ? output_path_
                                  : beam::CombinePaths(output_path_, dir);
  std::filesystem::create_directory(submap_output);

  Eigen::Matrix4d T_World_SubmapRef = submap_ref->T_WORLD_SUBMAP();
  Eigen::Matrix4d T_World_SubmapRef_Init = submap_ref->T_WORLD_SUBMAP_INIT();
  Eigen::Matrix4d T_World_SubmapTgt_Init = submap_tgt->T_WORLD_SUBMAP_INIT();

  // get initial relative pose
  Eigen::Matrix4d T_SubmapRef_SubmapTgt_Init =
      beam::InvertTransform(T_World_SubmapRef_Init) * T_World_SubmapTgt_Init;

  const auto T_W_B_i = submap_tgt->T_WORLD_SUBMAP();
  const bool use_initials = false;
  if (matcher_loam) {
    // first get maps in their initial world frame
    LoamPointCloudPtr ref_in_ref_submap_frame =
        std::make_shared<LoamPointCloud>(
            submap_ref->GetLidarLoamPointsInWorldFrame(use_initials));
    LoamPointCloudPtr tgt_in_ref_submap_frame =
        std::make_shared<LoamPointCloud>(
            submap_tgt->GetLidarLoamPointsInWorldFrame(use_initials));

    // then transform to the reference submap frame
    Eigen::Matrix4d T_SubmapRef_WorldInit =
        beam::InvertTransform(T_World_SubmapRef_Init);
    ref_in_ref_submap_frame->TransformPointCloud(T_SubmapRef_WorldInit);
    tgt_in_ref_submap_frame->TransformPointCloud(T_SubmapRef_WorldInit);

    // align
    matcher_loam->SetRef(ref_in_ref_submap_frame);
    matcher_loam->SetTarget(tgt_in_ref_submap_frame);
    bool match_success = matcher_loam->Match();
    Eigen::Matrix4d T_SubmapRef_SubmapTgt =
        matcher_loam->ApplyResult(T_SubmapRef_SubmapTgt_Init);
    Eigen::Matrix4d T_World_SubmapTgt =
        T_World_SubmapRef * T_SubmapRef_SubmapTgt;

    Eigen::Vector3d t_W_BS = submap_tgt->LidarKeyframes()
                                 .begin()
                                 ->second.T_REFFRAME_BASELINK()
                                 .block(0, 3, 3, 1);
    Eigen::Vector3d t_W_BE = submap_tgt->LidarKeyframes()
                                 .rbegin()
                                 ->second.T_REFFRAME_BASELINK()
                                 .block(0, 3, 3, 1);

    if (!output_path_.empty()) {
      PointCloud ref_in_world =
          submap_ref->GetLidarPointsInWorldFrameCombined(use_initials);
      PointCloud tgt_in_world =
          submap_tgt->GetLidarPointsInWorldFrameCombined(use_initials);

      std::string path_ref = beam::CombinePaths(submap_output, "reference.pcd");
      SaveViewableSubmap(path_ref, ref_in_world, 0, 0, 255, t_W_BS, t_W_BE);

      std::string path_init =
          beam::CombinePaths(submap_output, "target_initial.pcd");
      SaveViewableSubmap(path_init, tgt_in_world, 255, 0, 0, t_W_BS, t_W_BE);
    }

    // set new submap pose
    submap_tgt->UpdatePose(T_World_SubmapTgt);

    if (!output_path_.empty()) {
      PointCloud tgt_in_world_opt =
          submap_tgt->GetLidarPointsInWorldFrameCombined();
      std::string path_opt =
          beam::CombinePaths(submap_output, "target_aligned.pcd");
      SaveViewableSubmap(path_opt, tgt_in_world_opt, 0, 255, 0, t_W_BS, t_W_BE);
    }
  } else {
    // first get maps in their initial world frame
    PointCloud ref_in_world =
        submap_ref->GetLidarPointsInWorldFrameCombined(use_initials);
    PointCloud tgt_in_world =
        submap_tgt->GetLidarPointsInWorldFrameCombined(use_initials);

    // then transform to the reference submap frame
    Eigen::Matrix4d T_SubmapRef_WorldInit =
        beam::InvertTransform(T_World_SubmapRef_Init);
    PointCloudPtr ref_in_ref_submap_frame = std::make_shared<PointCloud>();
    PointCloudPtr tgt_in_ref_submap_frame = std::make_shared<PointCloud>();
    pcl::transformPointCloud(ref_in_world, *ref_in_ref_submap_frame,
                             T_SubmapRef_WorldInit.cast<float>());
    pcl::transformPointCloud(tgt_in_world, *tgt_in_ref_submap_frame,
                             T_SubmapRef_WorldInit.cast<float>());

    // align
    matcher->SetRef(ref_in_ref_submap_frame);
    matcher->SetTarget(tgt_in_ref_submap_frame);
    bool match_success = matcher->Match();
    Eigen::Matrix4d T_SubmapRef_SubmapTgt =
        matcher->ApplyResult(T_SubmapRef_SubmapTgt_Init);
    Eigen::Matrix4d T_World_SubmapTgt =
        T_World_SubmapRef * T_SubmapRef_SubmapTgt;

    Eigen::Vector3d t_W_BS = submap_tgt->LidarKeyframes()
                                 .begin()
                                 ->second.T_REFFRAME_BASELINK()
                                 .block(0, 3, 3, 1);
    Eigen::Vector3d t_W_BE = submap_tgt->LidarKeyframes()
                                 .rbegin()
                                 ->second.T_REFFRAME_BASELINK()
                                 .block(0, 3, 3, 1);

    if (!output_path_.empty()) {
      std::string path_ref = beam::CombinePaths(submap_output, "reference.pcd");
      SaveViewableSubmap(path_ref, ref_in_world, 0, 0, 255, t_W_BS, t_W_BE);

      std::string path_init =
          beam::CombinePaths(submap_output, "target_initial.pcd");
      SaveViewableSubmap(path_init, tgt_in_world, 255, 0, 0, t_W_BS, t_W_BE);
    }

    // set new submap pose
    submap_tgt->UpdatePose(T_World_SubmapTgt);

    if (!output_path_.empty()) {
      PointCloud tgt_in_world_opt =
          submap_tgt->GetLidarPointsInWorldFrameCombined();
      std::string path_opt =
          beam::CombinePaths(submap_output, "target_initial.pcd");
      SaveViewableSubmap(path_opt, tgt_in_world_opt, 0, 255, 0, t_W_BS, t_W_BE);
    }
  }

  const auto T_W_B_f = submap_tgt->T_WORLD_SUBMAP();
  RegistrationResult result(T_W_B_i, T_W_B_f);
  results_.emplace(submap_tgt->Stamp(), result);

  return true;
}

} // namespace bs_models::global_mapping