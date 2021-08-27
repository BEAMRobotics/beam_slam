#include <bs_models/frame_to_frame/scan_matcher_3d.h>

#include <boost/filesystem.hpp>
#include <fuse_core/transaction.h>
#include <pluginlib/class_list_macros.h>

#include <beam_filtering/VoxelDownsample.h>
#include <beam_matching/Matchers.h>
#include <beam_utils/filesystem.h>

#include <bs_models/scan_registration/multi_scan_registration.h>
#include <bs_models/scan_registration/scan_to_map_registration.h>
#include <bs_models/frame_initializers/frame_initializers.h>
#include <bs_common/bs_msgs.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(bs_models::frame_to_frame::ScanMatcher3D,
                       fuse_core::SensorModel)

namespace bs_models {
namespace frame_to_frame {

using namespace beam_matching;
using namespace scan_registration;

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
    scan_registration_ = std::make_unique<ScanToMapLoamRegistration>(
        std::move(matcher), params.GetBaseParams(), params.map_size,
        params.store_full_cloud);
    feature_extractor_ = std::make_shared<LoamFeatureExtractor>(matcher_params);
  } else if (params_.type == "MULTIICP") {
    std::unique_ptr<Matcher<PointCloudPtr>> matcher =
        std::make_unique<IcpMatcher>(
            IcpMatcher::Params(params_.matcher_params_path));
    MultiScanRegistrationBase::Params params;
    params.LoadFromJson(params_.registration_config_path);
    scan_registration_ = std::make_unique<MultiScanRegistration>(
        std::move(matcher), params.GetBaseParams(), params.num_neighbors,
        params.lag_duration, params.disable_lidar_map);
  } else if (params_.type == "MULTINDT") {
    std::unique_ptr<Matcher<PointCloudPtr>> matcher =
        std::make_unique<NdtMatcher>(
            NdtMatcher::Params(params_.matcher_params_path));
    MultiScanRegistrationBase::Params params;
    params.LoadFromJson(params_.registration_config_path);
    scan_registration_ = std::make_unique<MultiScanRegistration>(
        std::move(matcher), params.GetBaseParams(), params.num_neighbors,
        params.lag_duration, params.disable_lidar_map);
  } else if (params_.type == "MULTIGICP") {
    std::unique_ptr<Matcher<PointCloudPtr>> matcher =
        std::make_unique<GicpMatcher>(
            GicpMatcher::Params(params_.matcher_params_path));
    MultiScanRegistrationBase::Params params;
    params.LoadFromJson(params_.registration_config_path);
    scan_registration_ = std::make_unique<MultiScanRegistration>(
        std::move(matcher), params.GetBaseParams(), params.num_neighbors,
        params.lag_duration, params.disable_lidar_map);
  } else if (params_.type == "MULTILOAM") {
    std::shared_ptr<LoamParams> matcher_params =
        std::make_shared<LoamParams>(params_.matcher_params_path);
    std::unique_ptr<Matcher<LoamPointCloudPtr>> matcher =
        std::make_unique<LoamMatcher>(*matcher_params);
    MultiScanRegistrationBase::Params params;
    params.LoadFromJson(params_.registration_config_path);
    scan_registration_ = std::make_unique<MultiScanLoamRegistration>(
        std::move(matcher), params.GetBaseParams(), params.num_neighbors,
        params.lag_duration, params.disable_lidar_map);
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
    scan_registration_ = std::make_unique<ScanToMapLoamRegistration>(
        std::move(matcher), params.GetBaseParams(), params.map_size,
        params.store_full_cloud);
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
  subscriber_ = node_handle_.subscribe<sensor_msgs::PointCloud2>(
      ros::names::resolve(params_.input_topic), 1, &ThrottledCallback::callback,
      &throttled_callback_, ros::TransportHints().tcpNoDelay(false));

  results_publisher_ =
      private_node_handle_.advertise<SlamChunkMsg>(params_.output_topic, 100);
};

void ScanMatcher3D::onStop() {
  // if output set, save scans before stopping
  ROS_INFO("ScanMatcher3D stopped, processing remaining scans in window.");
  for (auto iter = active_clouds_.begin(); iter != active_clouds_.end();
       iter++) {
    OutputResults(*iter);
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

  Eigen::Matrix4d T_WORLD_BASELINKCURRENT;
  if (!frame_initializer_->GetEstimatedPose(T_WORLD_BASELINKCURRENT,
                                            msg->header.stamp,
                                            extrinsics_.GetBaselinkFrameId())) {
    ROS_DEBUG("Skipping scan");
    return bs_constraints::frame_to_frame::Pose3DStampedTransaction(
        msg->header.stamp);
  }

  Eigen::Matrix4d T_BASELINK_LIDAR;
  if (!extrinsics_.GetT_BASELINK_LIDAR(T_BASELINK_LIDAR, msg->header.stamp)) {
    ROS_ERROR(
        "Cannot get transform from lidar to baselink for stamp: %.8f. Skipping "
        "scan.",
        msg->header.stamp.toSec());
    return bs_constraints::frame_to_frame::Pose3DStampedTransaction(
        msg->header.stamp);
  }

  ScanPose current_scan_pose(*cloud_current, msg->header.stamp,
                             T_WORLD_BASELINKCURRENT, T_BASELINK_LIDAR,
                             feature_extractor_);

  active_clouds_.push_back(current_scan_pose);

  // build transaction of registration measurements
  auto transaction = scan_registration_->RegisterNewScan(current_scan_pose);

  return transaction;
}

void ScanMatcher3D::onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph_msg) {
  updates_++;

  auto i = active_clouds_.begin();
  while (i != active_clouds_.end()) {
    bool update_successful = i->UpdatePose(graph_msg);
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

  std::string update_time =
      beam::ConvertTimeToDate(std::chrono::system_clock::now());
  std::string curent_path = graph_updates_path_ + "U" +
                            std::to_string(updates_) + "_" + update_time + "/";
  boost::filesystem::create_directory(curent_path);
  for (auto iter = active_clouds_.begin(); iter != active_clouds_.end();
       iter++) {
    iter->SaveCloud(curent_path);
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

void ScanMatcher3D::OutputResults(const ScanPose& scan_pose) {
  if (!params_.output_topic.empty()) {
    // output to global mapper
    SlamChunkMsg slam_chunk_msg;
    slam_chunk_msg.stamp = scan_pose.Stamp();

    std::vector<float> pose;
    const Eigen::Matrix4d& T = scan_pose.T_REFFRAME_BASELINK();
    for (uint8_t i = 0; i < 3; i++) {
      for (uint8_t j = 0; j < 4; j++) {
        pose.push_back(static_cast<float>(T(i, j)));
      }
    }
    slam_chunk_msg.T_WORLD_BASELINK = pose;

    slam_chunk_msg.lidar_measurement.frame_id = extrinsics_.GetLidarFrameId();

    // publish regular points
    const PointCloud& cloud = scan_pose.Cloud();
    if (params_.output_lidar_points && cloud.size() > 0) {
      SlamChunkMsg points_msg = slam_chunk_msg;
      points_msg.lidar_measurement.point_type = 0;
      for (int i = 0; i < cloud.size(); i++) {
        pcl::PointXYZ p = cloud.points.at(i);
        points_msg.lidar_measurement.points.push_back(p.x);
        points_msg.lidar_measurement.points.push_back(p.y);
        points_msg.lidar_measurement.points.push_back(p.z);
      }
      ROS_DEBUG("Publishing lidar points");
      results_publisher_.publish(points_msg);
    }

    // publish loam pointcloud
    const beam_matching::LoamPointCloud& loam_cloud = scan_pose.LoamCloud();
    if (params_.output_loam_points && loam_cloud.Size() > 0) {
      // get strong edge features
      SlamChunkMsg edges_msg = slam_chunk_msg;
      edges_msg.lidar_measurement.point_type = 1;
      const PointCloud& edges = loam_cloud.edges.strong.cloud;
      for (int i = 0; i < edges.size(); i++) {
        pcl::PointXYZ p = edges.points.at(i);
        edges_msg.lidar_measurement.points.push_back(p.x);
        edges_msg.lidar_measurement.points.push_back(p.y);
        edges_msg.lidar_measurement.points.push_back(p.z);
      }
      ROS_DEBUG("Publishing loam edge points");
      results_publisher_.publish(edges_msg);

      // get strong surface features
      SlamChunkMsg surfaces_msg = slam_chunk_msg;
      surfaces_msg.lidar_measurement.point_type = 2;
      const PointCloud& surfaces = loam_cloud.surfaces.strong.cloud;
      for (int i = 0; i < surfaces.size(); i++) {
        pcl::PointXYZ p = surfaces.points.at(i);
        surfaces_msg.lidar_measurement.points.push_back(p.x);
        surfaces_msg.lidar_measurement.points.push_back(p.y);
        surfaces_msg.lidar_measurement.points.push_back(p.z);
      }
      ROS_DEBUG("Publishing loam surface points");
      results_publisher_.publish(surfaces_msg);
    }
  }

  // save to disk
  if (!params_.scan_output_directory.empty()) {
    scan_pose.SaveCloud(params_.scan_output_directory);
  }
}

}  // namespace frame_to_frame
}  // namespace bs_models
