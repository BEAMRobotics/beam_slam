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

namespace beam_models { namespace frame_to_frame {

ScanMatcher3D::ScanMatcher3D()
    : FrameToFrameSensorModelBase() {}

void ScanMatcher3D::onInit() {
  InitiateBaseClass(private_node_handle_);
  params_.loadExtraParams(private_node_handle_);

  // init scan registration
  std::unique_ptr<beam_matching::Matcher<PointCloudPtr>> matcher;
  if (params_.type == "ICP") {
    std::string config_path =
        beam::LibbeamRoot() + "beam_matching/config/icp.json";
    beam_matching::IcpMatcherParams matcher_params(config_path);
    matcher = std::make_unique<beam_matching::IcpMatcher>(matcher_params);
  } else if (params_.type == "GICP") {
    std::string config_path =
        beam::LibbeamRoot() + "beam_matching/config/gicp.json";
    beam_matching::GicpMatcherParams matcher_params(config_path);
    matcher = std::make_unique<beam_matching::GicpMatcher>(matcher_params);
  } else if (params_.type == "NDT") {
    std::string config_path =
        beam::LibbeamRoot() + "beam_matching/config/ndt.json";
    beam_matching::NdtMatcherParams matcher_params(config_path);
    matcher = std::make_unique<beam_matching::NdtMatcher>(matcher_params);
  } else {
    const std::string error =
        "scan matcher type invalid. Options: ICP, GICP, NDT.";
    ROS_FATAL_STREAM(error);
    throw std::runtime_error(error);
  }

  MultiScanRegistration::Params scan_reg_params{
      .num_neighbors = params_.num_neighbors,
      .outlier_threshold_t = params_.outlier_threshold_t,
      .outlier_threshold_r = params_.outlier_threshold_r,
      .source = name(),
      .lag_duration = params_.lag_duration,
      .fix_first_scan = params_.fix_first_scan};
  multi_scan_registration_ = std::make_unique<MultiScanRegistration>(
      std::move(matcher), scan_reg_params);

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
  PointCloudPtr cloud_current_tmp = beam::ROSToPCL(*msg);
  PointCloudPtr cloud_current = std::make_shared<PointCloud>();

  Eigen::Vector3f scan_voxel_size(params_.downsample_size,
                                  params_.downsample_size,
                                  params_.downsample_size);
  beam_filtering::VoxelDownsample downsampler(scan_voxel_size);
  downsampler.Filter(*cloud_current_tmp, *cloud_current);

  Eigen::Matrix4d T_WORLD_CLOUDCURRENT;
  if (!frame_initializer_->GetEstimatedPose(msg->header.stamp,
                                            T_WORLD_CLOUDCURRENT)) {
    return beam_constraints::frame_to_frame::Pose3DStampedTransaction(
        msg->header.stamp);
  }

  ScanPose current_scan_pose(msg->header.stamp, T_WORLD_CLOUDCURRENT,
                             *cloud_current);

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

  if (!output_graph_updates_) { return; }
  std::string curent_path = graph_updates_path_ + "U" +
                            std::to_string(updates_) + "_" + update_time + "/";
  boost::filesystem::create_directory(curent_path);
  boost::filesystem::create_directory(curent_path);
  for (auto iter = active_clouds_.begin(); iter != active_clouds_.end();
       iter++) {
    iter->Save(curent_path);
  }
}

}} // namespace beam_models::frame_to_frame
