#include <beam_models/frame_to_frame/scan_matcher_3d.h>

#include <fuse_core/transaction.h>
#include <pluginlib/class_list_macros.h>

#include <beam_matching/Matchers.h>
#include <beam_utils/filesystem.h>

#include <beam_common/sensor_proc.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(beam_models::frame_to_frame::ScanMatcher3D,
                       fuse_core::SensorModel)

namespace beam_models { namespace frame_to_frame {

ScanMatcher3D::ScanMatcher3D()
    : fuse_core::AsyncSensorModel(1), device_id_(fuse_core::uuid::NIL) {}

void ScanMatcher3D::onInit() {
  // Read settings from the parameter sever
  device_id_ = fuse_variables::loadDeviceId(private_node_handle_);
  params_.loadFromROS(private_node_handle_);

  if (params_.type == "ICP") {
    std::string config_path =
        beam::LibbeamRoot() + "beam_matching/config/icp.json";
    beam_matching::IcpMatcherParams matcher_params(config_path);
    matcher_ = std::make_unique<beam_matching::IcpMatcher>(matcher_params);
  } else if (params_.type == "GICP") {
    std::string config_path =
        beam::LibbeamRoot() + "beam_matching/config/gicp.json";
    beam_matching::GicpMatcherParams matcher_params(config_path);
    matcher_ = std::make_unique<beam_matching::GicpMatcher>(matcher_params);
  } else if (params_.type == "NDT") {
    std::string config_path =
        beam::LibbeamRoot() + "beam_matching/config/ndt.json";
    beam_matching::NdtMatcherParams matcher_params(config_path);
    matcher_ = std::make_unique<beam_matching::NdtMatcher>(matcher_params);
  } else {
    const std::string error =
        "scan matcher type invalid. Options: ICP, GICP, NDT.";
    ROS_FATAL_STREAM(error);
    throw std::runtime_error(error);
  }
}

void ScanMatcher3D::onStart() {
  reference_clouds_.clear();
  pointcloud_subscriber_ = node_handle_.subscribe(
      params_.pointcloud_topic, params_.queue_size,
      &PointCloudThrottledCallback::callback, &throttled_callback_);
}

void ScanMatcher3D::onStop() {
  pointcloud_subscriber_.shutdown();
}

void ScanMatcher3D::process(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  ReferenceCloud current_cloud;
  current_cloud.cloud = beam::ROSToPCL(*msg, current_cloud.time);
  // TODO: implement
  current_cloud.T_REF_CLOUD = GetEstimatedPose(current_cloud.time);

  // if first scan, add to list then exit
  if (reference_clouds_.empty()) {
    reference_clouds_.push_front(current_cloud);
    return;
  }

  for (auto iter = reference_clouds_.begin(); iter != reference_clouds_.end();
       iter++) {
    Eigen::Matrix4d T_CLOUD1_CLOUD2;
    Eigen::Matrix<double, 6, 6> covariance;
    MatchScans(*iter, current_cloud, T_CLOUD1_CLOUD2, covariance);
    current_cloud.T_REF_CLOUD = iter->T_REF_CLOUD * T_CLOUD1_CLOUD2;

    // Create a transaction object
    auto transaction = fuse_core::Transaction::make_shared();
    transaction->stamp(msg->header.stamp);

    // build transaction
    beam_common::processRelativePoseWithCovariance(
        name(), device_id_, iter->time, current_cloud.time, iter->T_REF_CLOUD,
        current_cloud.T_REF_CLOUD, covariance, *transaction);

    // Send the transaction object to the plugin's parent
    sendTransaction(transaction);
  }

  // add cloud to reference cloud list and remove last
  if (reference_clouds_.size() == params_.num_neighbors) {
    reference_clouds_.pop_back();
  }
  reference_clouds_.push_front(current_cloud);
}

void ScanMatcher3D::onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph_msg) {
  // TODO: get updated scan poses
}

Eigen::Matrix4d ScanMatcher3D::GetEstimatedPose(const ros::Time& time){
  // TODO
}

void ScanMatcher3D::MatchScans(const ReferenceCloud& cloud1,
                             const ReferenceCloud& cloud2,
                             Eigen::Matrix4d& T_CLOUD1_CLOUD2,
                             Eigen::Matrix<double, 6, 6>& covariance) {
  Eigen::Matrix4d T_CLOUD1EST_CLOUD2 =
      beam::InvertTransform(cloud1.T_REF_CLOUD) * cloud2.T_REF_CLOUD;

  // transform cloud2 into cloud1 frame
  PointCloudPtr cloud2_transformed;
  pcl::transformPointCloud(*cloud2.cloud, *cloud2_transformed,
                           Eigen::Affine3d(T_CLOUD1_CLOUD2));

  // match clouds
  matcher_->Setup(cloud1.cloud, cloud2_transformed);
  matcher_->Match();
  Eigen::Matrix4d T_CLOUD1_CLOUD1EST = matcher_->GetResult().matrix();
  T_CLOUD1_CLOUD2 = T_CLOUD1_CLOUD1EST * T_CLOUD1EST_CLOUD2;
  covariance = matcher_->GetInfo();
}

}} // namespace beam_models::frame_to_frame
