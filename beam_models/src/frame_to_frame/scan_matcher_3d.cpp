#include <beam_models/frame_to_frame/scan_matcher_3d.h>

#include <fuse_core/transaction.h>
#include <pluginlib/class_list_macros.h>

#include <beam_matching/Matchers.h>
#include <beam_utils/filesystem.h>

#include <beam_common/sensor_proc.h>
#include <beam_models/frame_initializers/frame_initializers.h>

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

  // init matcher
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

  // init frame initializer
  if (params_.frame_initializer_type == "ODOMETRY") {
    ros::Subscriber odometry_subscriber =
        private_node_handle_.subscribe<nav_msgs::Odometry>(
            params_.frame_initializer_topic, 100,
            boost::bind(
                &frame_initializers::OdometryFrameInitializer::OdometryCallback,
                &frame_initializer_, _1));
    // frame_initializer_ =
    //     std::make_unique<frame_initializers::OdometryFrameInitializer>(
    //         private_node_handle_, params_.frame_initializer_topic,
    //         params_.pointcloud_frame);
    frame_initializer_ =
        std::make_unique<frame_initializers::OdometryFrameInitializer>(
            odometry_subscriber, params_.pointcloud_frame);
  } else {
    const std::string error =
        "frame_initializer_type invalid. Options: ODOMETRY";
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
  PointCloudPtr cloud_current = beam::ROSToPCL(*msg);

  Eigen::Matrix4d T_WORLD_CLOUDCURRENT;
  if (!frame_initializer_->GetEstimatedPose(msg->header.stamp,
                                            T_WORLD_CLOUDCURRENT)) {
    return;
  }

  ScanPose current_scan_pose(msg->header.stamp, T_WORLD_CLOUDCURRENT,
                             cloud_current);

  // if first scan, add to list then exit
  if (reference_clouds_.empty()) {
    reference_clouds_.push_front(current_scan_pose);
    return;
  }

  for (auto iter = reference_clouds_.begin(); iter != reference_clouds_.end();
       iter++) {
    // run matcher to get refined cloud pose
    Eigen::Matrix4d T_CLOUDREF_CLOUDCURRENT;
    Eigen::Matrix<double, 6, 6> covariance;
    MatchScans(iter->Cloud(), cloud_current, iter->T_WORLD_CLOUD(),
               T_WORLD_CLOUDCURRENT, T_CLOUDREF_CLOUDCURRENT, covariance);

    // update estimate of cloud pose
    T_WORLD_CLOUDCURRENT = iter->T_WORLD_CLOUD() * T_CLOUDREF_CLOUDCURRENT;

    // Create a transaction object
    auto transaction = fuse_core::Transaction::make_shared();
    transaction->stamp(msg->header.stamp);
    beam_common::processRelativePoseWithCovariance(
        name(), iter->Position(), iter->Orientation(),
        current_scan_pose.Position(), current_scan_pose.Orientation(),
        T_CLOUDREF_CLOUDCURRENT, covariance, *transaction);

    // Send the transaction object to the plugin's parent
    sendTransaction(transaction);
  }

  // add cloud to reference cloud list and remove last
  if (reference_clouds_.size() == params_.num_neighbors) {
    reference_clouds_.pop_back();
  }
  reference_clouds_.push_front(current_scan_pose);
}

void ScanMatcher3D::onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph_msg) {
  // update scan poses
  for (auto iter = reference_clouds_.begin(); iter != reference_clouds_.end();
       iter++) {
    iter->Update(graph_msg);
  }
}

void ScanMatcher3D::MatchScans(const PointCloudPtr& cloud1,
                               const PointCloudPtr& cloud2,
                               const Eigen::Matrix4d& T_WORLD_CLOUD1,
                               const Eigen::Matrix4d& T_WORLD_CLOUD2,
                               Eigen::Matrix4d& T_CLOUD1_CLOUD2,
                               Eigen::Matrix<double, 6, 6>& covariance) {
  Eigen::Matrix4d T_CLOUD1EST_CLOUD2 =
      beam::InvertTransform(T_WORLD_CLOUD1) * T_WORLD_CLOUD2;

  // transform cloud2 into cloud1 frame
  PointCloudPtr cloud2_transformed;
  pcl::transformPointCloud(*cloud2, *cloud2_transformed,
                           Eigen::Affine3d(T_CLOUD1EST_CLOUD2));

  // match clouds
  matcher_->Setup(cloud1, cloud2_transformed);
  matcher_->Match();
  Eigen::Matrix4d T_CLOUD1_CLOUD1EST = matcher_->GetResult().matrix();
  T_CLOUD1_CLOUD2 = T_CLOUD1_CLOUD1EST * T_CLOUD1EST_CLOUD2;
  covariance = matcher_->GetInfo();
}

}} // namespace beam_models::frame_to_frame
