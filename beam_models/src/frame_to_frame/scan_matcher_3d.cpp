#include <beam_models/frame_to_frame/scan_matcher_3d.h>

#include <boost/filesystem.hpp>
#include <fuse_core/transaction.h>
#include <pluginlib/class_list_macros.h>

#include <beam_filtering/VoxelDownsample.h>
#include <beam_matching/Matchers.h>
#include <beam_utils/filesystem.h>

#include <beam_models/frame_initializers/frame_initializers.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(beam_models::frame_to_frame::ScanMatcher3D,
                       fuse_core::SensorModel)

namespace beam_models {
namespace frame_to_frame {

using namespace beam_matching;

ScanMatcher3D::ScanMatcher3D() : FrameToFrameSensorModelBase() {}

void ScanMatcher3D::onInit() {
  InitiateBaseClass(private_node_handle_);
  params_.loadExtraParams(private_node_handle_);

  MultiScanRegistration::Params scan_reg_params{
      .num_neighbors = params_.num_neighbors,
      .outlier_threshold_t = params_.outlier_threshold_t,
      .outlier_threshold_r = params_.outlier_threshold_r,
      .min_motion_trans_m = params_.min_motion_trans_m,
      .min_motion_rot_rad = params_.min_motion_rot_rad,
      .source = name(),
      .lag_duration = params_.lag_duration,
      .fix_first_scan = params_.fix_first_scan};

  // init scan registration
  std::string default_config_path =
      beam::LibbeamRoot() + "beam_matching/config/";
  if (params_.type == "ICP") {
    IcpMatcherParams matcher_params;
    if (params_.matcher_params_path.empty()) {
      matcher_params = IcpMatcherParams(default_config_path += "icp.json");
    } else {
      matcher_params = IcpMatcherParams(params_.matcher_params_path);
    }
    std::unique_ptr<Matcher<PointCloudPtr>> matcher;
    matcher = std::make_unique<IcpMatcher>(matcher_params);
    multi_scan_registration_ = std::make_unique<MultiScanRegistration>(
        std::move(matcher), scan_reg_params);
  } else if (params_.type == "GICP") {
    GicpMatcherParams matcher_params;
    if (params_.matcher_params_path.empty()) {
      matcher_params = GicpMatcherParams(default_config_path += "gicp.json");
    } else {
      matcher_params = GicpMatcherParams(params_.matcher_params_path);
    }
    std::unique_ptr<Matcher<PointCloudPtr>> matcher;
    matcher = std::make_unique<GicpMatcher>(matcher_params);
    multi_scan_registration_ = std::make_unique<MultiScanRegistration>(
        std::move(matcher), scan_reg_params);
  } else if (params_.type == "NDT") {
    NdtMatcherParams matcher_params;
    if (params_.matcher_params_path.empty()) {
      matcher_params = NdtMatcherParams(default_config_path += "ndt.json");
    } else {
      matcher_params = NdtMatcherParams(params_.matcher_params_path);
    }
    std::unique_ptr<Matcher<PointCloudPtr>> matcher;
    matcher = std::make_unique<NdtMatcher>(matcher_params);
    multi_scan_registration_ = std::make_unique<MultiScanRegistration>(
        std::move(matcher), scan_reg_params);
  } else if (params_.type == "LOAM") {
    std::shared_ptr<LoamParams> matcher_params;
    if (params_.matcher_params_path.empty()) {
      matcher_params =
          std::make_shared<LoamParams>(default_config_path += "loam.json");
    } else {
      matcher_params =
          std::make_shared<LoamParams>(params_.matcher_params_path);
    }
    std::unique_ptr<Matcher<LoamPointCloudPtr>> matcher;
    matcher = std::make_unique<LoamMatcher>(*matcher_params);
    feature_extractor_ = std::make_shared<LoamFeatureExtractor>(matcher_params);
    multi_scan_registration_ = std::make_unique<MultiScanLoamRegistration>(
        std::move(matcher), scan_reg_params);
  } else {
    const std::string error =
        "scan matcher type invalid. Options: ICP, GICP, NDT, LOAM";
    ROS_FATAL_STREAM(error);
    throw std::runtime_error(error);
  }

  // set covariance if not set to zero in config
  if (std::accumulate(params_.matcher_noise_diagonal.begin(),
                      params_.matcher_noise_diagonal.end(), 0.0) > 0) {
    Eigen::Matrix<double, 6, 6> covariance;
    covariance.setIdentity();
    for (int i = 0; i < 6; i++) {
      covariance(i, i) = params_.matcher_noise_diagonal[i];
    }
    multi_scan_registration_->SetFixedCovariance(covariance);
  }

  // if outputting scans, clear folder
  if (!params_.scan_output_directory.empty()) {
    if (boost::filesystem::is_directory(params_.scan_output_directory)) {
      boost::filesystem::remove_all(params_.scan_output_directory);
    }
    boost::filesystem::create_directory(params_.scan_output_directory);
  }

  // if outputting graph update results, clear results folder:
  if (output_graph_updates_) {
    if (boost::filesystem::is_directory(graph_updates_path_)) {
      boost::filesystem::remove_all(graph_updates_path_);
    }
    boost::filesystem::create_directory(graph_updates_path_);
  }
}

void ScanMatcher3D::onStop() {
  // if output set, save scans before stopping
  if (!params_.scan_output_directory.empty()) {
    ROS_DEBUG("Saving remaining scans in window to %d",
              params_.scan_output_directory.c_str());
    for (auto iter = active_clouds_.begin(); iter != active_clouds_.end();
         iter++) {
      iter->Save(params_.scan_output_directory);
    }
  }
  active_clouds_.clear();
  subscriber_.shutdown();
}

beam_constraints::frame_to_frame::Pose3DStampedTransaction
ScanMatcher3D::GenerateTransaction(
    const sensor_msgs::PointCloud2::ConstPtr& msg) {
  ROS_DEBUG("Received incoming scan");
  PointCloudPtr cloud_current = beam::ROSToPCL(*msg);

  if (params_.downsample_size > 0) {
    Eigen::Vector3f scan_voxel_size(params_.downsample_size,
                                    params_.downsample_size,
                                    params_.downsample_size);
    beam_filtering::VoxelDownsample downsampler(scan_voxel_size);
    downsampler.Filter(*cloud_current, *cloud_current);
  }

  Eigen::Matrix4d T_WORLD_CLOUDCURRENT;
  if (!frame_initializer_->GetEstimatedPose(msg->header.stamp,
                                            T_WORLD_CLOUDCURRENT)) {
    return beam_constraints::frame_to_frame::Pose3DStampedTransaction(
        msg->header.stamp);
  }

  ScanPose current_scan_pose(msg->header.stamp, T_WORLD_CLOUDCURRENT,
                             *cloud_current, feature_extractor_);

  // if outputting scans, add to the active list
  if (!params_.scan_output_directory.empty() || output_graph_updates_) {
    active_clouds_.push_back(current_scan_pose);
  }

  // build transaction of registration measurements
  return multi_scan_registration_->RegisterNewScan(current_scan_pose);
}

void ScanMatcher3D::onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph_msg) {
  updates_++;

  std::string update_time =
      beam::ConvertTimeToDate(std::chrono::system_clock::now());

  auto i = active_clouds_.begin();
  while (i != active_clouds_.end()) {
    bool update_successful = i->Update(graph_msg);
    if (update_successful) {
      ++i;
      continue;
    }

    // if scan has never been updated, then it is probably just not yet in the
    // window so do nothing
    if (i->Updates() == 0) {
      ++i;
      continue;
    }

    // Othewise, it has probably been marginalized out, so save and remove from
    // active list
    if (!params_.scan_output_directory.empty()) {
      i->Save(params_.scan_output_directory);
    }
    active_clouds_.erase(i++);
  }

  if (!output_graph_updates_) {
    return;
  }
  std::string curent_path = graph_updates_path_ + "U" +
                            std::to_string(updates_) + "_" + update_time + "/";
  boost::filesystem::create_directory(curent_path);
  boost::filesystem::create_directory(curent_path);
  for (auto iter = active_clouds_.begin(); iter != active_clouds_.end();
       iter++) {
    iter->Save(curent_path);
  }
}

}  // namespace frame_to_frame
}  // namespace beam_models
