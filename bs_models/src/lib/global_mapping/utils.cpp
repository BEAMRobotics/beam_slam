#include <bs_models/global_mapping/utils.h>

#include <beam_filtering/CropBox.h>
#include <beam_filtering/VoxelDownsample.h>

namespace bs_models::global_mapping {

const Eigen::Vector3f OUTPUT_VOX{0.04, 0.04, 0.04};
const Eigen::Vector3f OUTPUT_CROP_MIN{-30, -30, -10};
const Eigen::Vector3f OUTPUT_CROP_MAX{30, 30, 10};

RegistrationResult::RegistrationResult(const Eigen::Matrix4d& Ti,
                                       const Eigen::Matrix4d& Tf) {
  Eigen::Matrix4d T_DIFF = beam::InvertTransform(Ti) * Tf;
  Eigen::Matrix3d R_DIFF = T_DIFF.block(0, 0, 3, 3);
  dR = beam::Rad2Deg(std::abs(Eigen::AngleAxis<double>(R_DIFF).angle()));
  dt = T_DIFF.block(0, 3, 3, 1).norm() * 1000;
}

void SaveViewableSubmap(const std::string& save_path, const PointCloud& cloud,
                        uint8_t r, uint8_t g, uint8_t b,
                        const Eigen::Vector3d& t_world_BaselinkStart,
                        const Eigen::Vector3d& t_world_BaselinkEnd) {
  auto cloud_ptr = std::make_shared<PointCloud>(cloud);
  beam_filtering::CropBox<pcl::PointXYZ> cropper;
  Eigen::Vector3f crop_min =
      OUTPUT_CROP_MIN + t_world_BaselinkStart.cast<float>();
  Eigen::Vector3f crop_max =
      OUTPUT_CROP_MAX + t_world_BaselinkEnd.cast<float>();
  cropper.SetMinVector(crop_min);
  cropper.SetMaxVector(crop_max);
  cropper.SetRemoveOutsidePoints(true);
  cropper.SetInputCloud(cloud_ptr);
  cropper.Filter();
  auto pc_cropped = std::make_shared<PointCloud>();
  *pc_cropped = cropper.GetFilteredCloud();

  beam_filtering::VoxelDownsample voxel_filter(OUTPUT_VOX);
  voxel_filter.SetInputCloud(pc_cropped);
  voxel_filter.Filter();
  PointCloud pc_downsampled = voxel_filter.GetFilteredCloud();

  PointCloudCol pc_col = beam::ColorPointCloud(pc_downsampled, r, g, b);
  beam::SavePointCloud<pcl::PointXYZRGB>(save_path, pc_col);
}

void UpdateSubmapScanPosesFromGraph(
    std::vector<SubmapPtr> submaps,
    std::shared_ptr<fuse_graphs::HashGraph> graph, uint64_t max_timestamp) {
  for (auto& submap : submaps) {
    for (auto scan_iter = submap->LidarKeyframesBegin();
         scan_iter != submap->LidarKeyframesEnd(); scan_iter++) {
      scan_iter->second.UpdatePose(graph);
      if (max_timestamp != 0 && scan_iter->first >= max_timestamp) { return; }
    }
  }
}

void UpdateSubmapPosesFromGraph(std::vector<SubmapPtr> submaps,
                                std::shared_ptr<fuse_graphs::HashGraph> graph) {
  for (uint16_t i = 0; i < submaps.size(); i++) {
    bool success = submaps.at(i)->UpdatePose(graph);
    if (success) {
      BEAM_INFO("Updated submap {} pose from graph", i);
    } else {
      BEAM_INFO("Submap {} pose not in the graph", i);
    }
  }
}

} // namespace bs_models::global_mapping