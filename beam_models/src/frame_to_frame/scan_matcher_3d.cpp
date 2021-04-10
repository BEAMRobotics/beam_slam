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
    : fuse_core::AsyncSensorModel(1),
      device_id_(fuse_core::uuid::NIL),
      throttled_callback_(
          std::bind(&ScanMatcher3D::process, this, std::placeholders::_1)) {}

void ScanMatcher3D::onInit() {
  // Read settings from the parameter sever
  device_id_ = fuse_variables::loadDeviceId(private_node_handle_);
  params_.loadFromROS(private_node_handle_);

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

  multi_scan_registration_ = std::make_unique<MultiScanRegistration>(
      std::move(matcher), params_.num_neighbors, params_.outlier_threshold_t,
      params_.outlier_threshold_r, name(), params_.fix_first_scan);

  // Eigen::Matrix<double, 6, 1> cov_dia;
  // cov_dia << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
  // Eigen::Matrix<double, 6, 6> covariance = cov_dia.asDiagonal();
  // multi_scan_registration_->SetFixedCovariance(covariance);

  // init frame initializer
  if (params_.frame_initializer_type == "ODOMETRY") {
    frame_initializer_ =
        std::make_unique<frame_initializers::OdometryFrameInitializer>(
            params_.frame_initializer_topic, 100, params_.pointcloud_frame,
            true, 30);
  } else {
    const std::string error =
        "frame_initializer_type invalid. Options: ODOMETRY";
    ROS_FATAL_STREAM(error);
    throw std::runtime_error(error);
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

void ScanMatcher3D::onStart() {
  pointcloud_subscriber_ = node_handle_.subscribe(
      params_.pointcloud_topic, params_.queue_size,
      &PointCloudThrottledCallback::callback, &throttled_callback_);
}

void ScanMatcher3D::onStop() {
  // if output set, save scans before stopping
  if (!params_.scan_output_directory.empty()) {
    ROS_DEBUG("Saving remaining scans in window to %d",
              params_.scan_output_directory.c_str());
    for (auto iter = active_clouds_.begin(); iter != active_clouds_.end();
         iter++) {
      iter->second->Save(params_.scan_output_directory);
    }
  }
  active_clouds_.clear();
  pointcloud_subscriber_.shutdown();
}

void ScanMatcher3D::process(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  ROS_DEBUG("Received incoming scan");
  PointCloudPtr cloud_current_tmp = beam::ROSToPCL(*msg);
  PointCloudPtr cloud_current = boost::make_shared<PointCloud>();

  Eigen::Vector3f scan_voxel_size(params_.downsample_size,
                                  params_.downsample_size,
                                  params_.downsample_size);
  beam_filtering::VoxelDownsample downsampler(scan_voxel_size);
  downsampler.Filter(*cloud_current_tmp, *cloud_current);

  Eigen::Matrix4d T_WORLD_CLOUDCURRENT;
  if (!frame_initializer_->GetEstimatedPose(msg->header.stamp,
                                            T_WORLD_CLOUDCURRENT)) {
    return;
  }

  std::shared_ptr<ScanPose> current_scan_pose = std::make_shared<ScanPose>(
      msg->header.stamp, T_WORLD_CLOUDCURRENT, cloud_current);

  // if outputting scans, add to the active list
  if (!params_.scan_output_directory.empty() || output_graph_updates_) {
    active_clouds_.emplace(
        boost::to_string(current_scan_pose->Position()->uuid()),
        current_scan_pose);
  }

  // build transaction of registration measurements
  fuse_core::Transaction::SharedPtr transaction =
      multi_scan_registration_->RegisterNewScan(current_scan_pose);

  // Send the transaction object to the plugin's parent
  if (transaction != nullptr) {
    ROS_DEBUG("Sending transaction");
    sendTransaction(transaction);
  }
}

// TODO: active_clouds_ shouldn't need to be a map since we aren't searching
// anymore. It is useful for removing specific items but not the best
// implementation
void ScanMatcher3D::onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph_msg) {
  std::string update_time =
      beam::ConvertTimeToDate(std::chrono::system_clock::now());

  // update only reference clouds if we are not storing all clouds in the graph
  if (params_.scan_output_directory.empty() && !output_graph_updates_) {
    multi_scan_registration_->UpdateScanPoses(graph_msg);
    return;
  }

  // otherwise, iterate through all cloud poses in the graph and update
  for (auto iter = active_clouds_.begin(); iter != active_clouds_.end();
       iter++) {
    bool update_successful = iter->second->Update(graph_msg);

    if (update_successful) { continue; }

    // if scan has never been updated, then it is probably just not yet in the
    // window so do nothing
    if (iter->second->Updates() == 0) { continue; }

    // Othewise, it has probably been marginalized out, so save and remove from
    // active list
    if (!params_.scan_output_directory.empty()) {
      iter->second->Save(params_.scan_output_directory);
    }
    active_clouds_.erase(iter->first);
  }

  updates_++;

  std::string curent_path = graph_updates_path_ + "U" +
                            std::to_string(updates_) + "_" + update_time + "/";
  boost::filesystem::create_directory(curent_path);
  boost::filesystem::create_directory(curent_path);
  for (auto iter = active_clouds_.begin(); iter != active_clouds_.end();
       iter++) {
    iter->second->Save(curent_path);
  }
}

}} // namespace beam_models::frame_to_frame
