#include <bs_models/frame_to_frame/scan_matcher_3d.h>

#include <boost/filesystem.hpp>
#include <fuse_core/transaction.h>
#include <pluginlib/class_list_macros.h>

#include <beam_filtering/VoxelDownsample.h>
#include <beam_matching/Matchers.h>
#include <beam_utils/filesystem.h>

#include <bs_models/frame_to_frame/scan_registration/multi_scan_registration.h>
#include <bs_models/frame_to_frame/scan_registration/scan_to_map_registration.h>
#include <bs_models/frame_initializers/frame_initializers.h>
#include <bs_common/SlamChunkMsg.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(bs_models::frame_to_frame::ScanMatcher3D,
                       fuse_core::SensorModel)

namespace bs_models {
namespace frame_to_frame {

using namespace beam_matching;

ScanMatcher3D::ScanMatcher3D()
    : fuse_core::AsyncSensorModel(1),
      device_id_(fuse_core::uuid::NIL),
      throttled_callback_(
          std::bind(&ScanMatcher3D::process, this, std::placeholders::_1)) {}

void ScanMatcher3D::onInit() {
  params_.loadFromROS(private_node_handle_);

  // init frame initializer
  if (params_.frame_initializer_type == "ODOMETRY") {
    frame_initializer_ =
        std::make_unique<frame_initializers::OdometryFrameInitializer>(
            params_.frame_initializer_info, 100, 30,
            params_.sensor_frame_id_override);
  } else if (params_.frame_initializer_type == "POSEFILE") {
    frame_initializer_ =
        std::make_unique<frame_initializers::PoseFileFrameInitializer>(
            params_.frame_initializer_info);
  } else {
    const std::string error =
        "frame_initializer_type invalid. Options: ODOMETRY, POSEFILE";
    ROS_FATAL_STREAM(error);
    throw std::runtime_error(error);
  }

  // init scan registration
  if (params_.type == "MAPLOAM") {
    std::shared_ptr<LoamParams> matcher_params =
        std::make_shared<LoamParams>(params_.matcher_params_path);
    std::unique_ptr<Matcher<LoamPointCloudPtr>> matcher =
        std::make_unique<LoamMatcher>(*matcher_params);
    ScanToMapLoamRegistration::Params params;
    params.LoadFromJson(params_.registration_config_path);
    scan_registration_ =
        std::make_unique<ScanToMapLoamRegistration>(std::move(matcher), params);
    feature_extractor_ = std::make_shared<LoamFeatureExtractor>(matcher_params);
  } else if (params_.type == "MULTIICP") {
    std::unique_ptr<Matcher<PointCloudPtr>> matcher =
        std::make_unique<IcpMatcher>(
            IcpMatcher::Params(params_.matcher_params_path));
    MultiScanRegistrationBase::Params params;
    params.LoadFromJson(params_.registration_config_path);
    scan_registration_ =
        std::make_unique<MultiScanRegistration>(std::move(matcher), params);
  } else if (params_.type == "MULTINDT") {
    std::unique_ptr<Matcher<PointCloudPtr>> matcher =
        std::make_unique<NdtMatcher>(
            NdtMatcher::Params(params_.matcher_params_path));
    MultiScanRegistrationBase::Params params;
    params.LoadFromJson(params_.registration_config_path);
    scan_registration_ =
        std::make_unique<MultiScanRegistration>(std::move(matcher), params);
  } else if (params_.type == "MULTIGICP") {
    std::unique_ptr<Matcher<PointCloudPtr>> matcher =
        std::make_unique<GicpMatcher>(
            GicpMatcher::Params(params_.matcher_params_path));
    MultiScanRegistrationBase::Params params;
    params.LoadFromJson(params_.registration_config_path);
    scan_registration_ =
        std::make_unique<MultiScanRegistration>(std::move(matcher), params);
  } else if (params_.type == "MULTILOAM") {
    std::shared_ptr<LoamParams> matcher_params =
        std::make_shared<LoamParams>(params_.matcher_params_path);
    std::unique_ptr<Matcher<LoamPointCloudPtr>> matcher =
        std::make_unique<LoamMatcher>(*matcher_params);
    MultiScanRegistrationBase::Params params;
    params.LoadFromJson(params_.registration_config_path);
    scan_registration_ =
        std::make_unique<MultiScanLoamRegistration>(std::move(matcher), params);
    feature_extractor_ = std::make_shared<LoamFeatureExtractor>(matcher_params);
  } else {
    BEAM_ERROR(
        "Invalid scan matcher 3d type. Input: {}, Using default: MAPLOAM",
        params_.type);
    std::shared_ptr<LoamParams> matcher_params =
        std::make_shared<LoamParams>(params_.matcher_params_path);
    std::unique_ptr<Matcher<LoamPointCloudPtr>> matcher =
        std::make_unique<LoamMatcher>(*matcher_params);
    ScanToMapLoamRegistration::Params params;
    params.LoadFromJson(params_.registration_config_path);
    scan_registration_ =
        std::make_unique<ScanToMapLoamRegistration>(std::move(matcher), params);
    feature_extractor_ = std::make_shared<LoamFeatureExtractor>(matcher_params);
  }

  // set covariance if not set to zero in config
  if (std::accumulate(params_.matcher_noise_diagonal.begin(),
                      params_.matcher_noise_diagonal.end(), 0.0) > 0) {
    Eigen::Matrix<double, 6, 6> covariance;
    covariance.setIdentity();
    for (int i = 0; i < 6; i++) {
      covariance(i, i) = params_.matcher_noise_diagonal[i];
    }
    scan_registration_->SetFixedCovariance(covariance);
  } else {
    scan_registration_->SetFixedCovariance(params_.matcher_noise);
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
  subscriber_ = node_handle_.subscribe(params_.input_topic, 1,
                                       &ThrottledCallback::callback,
                                       &throttled_callback_);

  results_publisher_ =
      private_node_handle_.advertise<bs_common::SlamChunkMsg>(
          params_.output_topic, 100);
};

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

bs_constraints::frame_to_frame::Pose3DStampedTransaction
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
  if (!frame_initializer_->GetEstimatedPose(T_WORLD_CLOUDCURRENT,
                                            msg->header.stamp,
                                            extrinsics_.GetLidarFrameId())) {
    ROS_DEBUG("Skipping scan");
    return bs_constraints::frame_to_frame::Pose3DStampedTransaction(
        msg->header.stamp);
  }

  bs_common::ScanPose current_scan_pose(
      msg->header.stamp, T_WORLD_CLOUDCURRENT, extrinsics_.GetWorldFrameId(),
      extrinsics_.GetLidarFrameId(), *cloud_current, feature_extractor_);

  // if outputting scans, add to the active list
  if (!params_.scan_output_directory.empty() || output_graph_updates_) {
    active_clouds_.push_back(current_scan_pose);
  }

  // build transaction of registration measurements
  return scan_registration_->RegisterNewScan(current_scan_pose);
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

    // Othewise, it has probably been marginalized out, so output and remove
    // from active list
    OutputResults(*i);
    active_clouds_.erase(i++);
  }

  if (!output_graph_updates_) {
    return;
  }
  std::string curent_path = graph_updates_path_ + "U" +
                            std::to_string(updates_) + "_" + update_time + "/";
  boost::filesystem::create_directory(curent_path);
  for (auto iter = active_clouds_.begin(); iter != active_clouds_.end();
       iter++) {
    iter->Save(curent_path);
  }
}

void ScanMatcher3D::process(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  bs_constraints::frame_to_frame::Pose3DStampedTransaction new_transaction =
      GenerateTransaction(msg);
  if (new_transaction.GetTransaction() != nullptr) {
    ROS_DEBUG("Sending transaction.");
    try {
      sendTransaction(new_transaction.GetTransaction());
    } catch (const std::exception& e) {
      ROS_WARN("Cannot send transaction. Error: %s", e.what());
    }
  }
}

void ScanMatcher3D::OutputResults(const bs_common::ScanPose& scan_pose) {
  // output to global mapper
  bs_common::SlamChunkMsg slam_chunk_msg;
  slam_chunk_msg.stamp = scan_pose.Stamp();

  std::vector<float> pose;
  const Eigen::Matrix4d& T = scan_pose.T_REFFRAME_CLOUD();
  for (uint8_t i = 0; i < 3; i++) {
    for (uint8_t j = 0; j < 4; j++) {
      pose.push_back(static_cast<float>(T(i, j)));
    }
  }
  slam_chunk_msg.T_WORLD_BASELINK = pose;
  slam_chunk_msg.baselink_frame_id = extrinsics_.GetBaselinkFrameId();

  // TODO: add scan data in baselink frame (?)

  // save to disk
  if (!params_.scan_output_directory.empty()) {
    scan_pose.Save(params_.scan_output_directory);
  }
}

}  // namespace frame_to_frame
}  // namespace bs_models
