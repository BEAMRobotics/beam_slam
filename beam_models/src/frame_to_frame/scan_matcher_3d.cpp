#include <beam_models/frame_to_frame/scan_matcher_3d.h>

#include <fuse_core/transaction.h>
#include <pluginlib/class_list_macros.h>

#include <beam_filtering/VoxelDownsample.h>
#include <beam_matching/Matchers.h>
#include <beam_utils/filesystem.h>

#include <beam_common/sensor_proc.h>
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
}

void ScanMatcher3D::onStart() {
  reference_clouds_.clear();
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
  reference_clouds_.clear();
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
  if (!params_.scan_output_directory.empty()) {
    active_clouds_.emplace(
        boost::to_string(current_scan_pose->Position()->uuid()),
        current_scan_pose);
  }

  // if first scan, add to list then exit
  if (reference_clouds_.empty()) {
    reference_clouds_.push_front(current_scan_pose);
    return;
  }

  int counter = 0;
  for (auto iter = reference_clouds_.begin(); iter != reference_clouds_.end();
       iter++) {
    counter++;
    ROS_DEBUG("Matching against neighbor no. %d", counter);

    // run matcher to get refined cloud pose
    Eigen::Matrix4d T_CLOUDREF_CLOUDCURRENT;
    Eigen::Matrix<double, 6, 6> covariance;
    MatchScans((*iter)->Cloud(), cloud_current, (*iter)->T_WORLD_CLOUD(),
               T_WORLD_CLOUDCURRENT, T_CLOUDREF_CLOUDCURRENT, covariance);

    if(!PassedThreshold(T_CLOUDREF_CLOUDCURRENT, beam::InvertTransform((*iter)->T_WORLD_CLOUD()) * T_WORLD_CLOUDCURRENT)){
      ROS_DEBUG("Failed scan matcher transform threshold check. Skipping measurement.");
      continue;
    }

    /*
    /// DELETE ME
    PointCloudPtr cloud_ref = (*iter)->Cloud();
    PointCloud cloud_ref_world;
    PointCloud cloud_cur_initial_world;
    PointCloud cloud_cur_aligned_world;
    const Eigen::Matrix4d T_WORLD_CLOUD_REF_INIT =
        (*iter)->T_WORLD_CLOUD_INIT();
    Eigen::Matrix4d T_WORLD_CLOUD_CURRENT_INIT = T_WORLD_CLOUDCURRENT;

    Eigen::Matrix4d T_CLOUDREF_CLOUDCURRENT_TMP;
    Eigen::Matrix<double, 6, 6> tmp;
    MatchScans(cloud_ref, cloud_current, T_WORLD_CLOUD_REF_INIT,
               T_WORLD_CLOUD_CURRENT_INIT, T_CLOUDREF_CLOUDCURRENT_TMP, tmp);        

    Eigen::Matrix4d T_WORLD_CLOUDCUR_ALIGNED =
        T_WORLD_CLOUD_REF_INIT * T_CLOUDREF_CLOUDCURRENT_TMP;
    pcl::transformPointCloud(*cloud_ref, cloud_ref_world,
                             T_WORLD_CLOUD_REF_INIT);
    pcl::transformPointCloud(*cloud_current, cloud_cur_initial_world,
                             T_WORLD_CLOUD_CURRENT_INIT);
    pcl::transformPointCloud(*cloud_current, cloud_cur_aligned_world,
                             T_WORLD_CLOUDCUR_ALIGNED);
    pcl::io::savePCDFileASCII("/home/nick/tmp/" +
                                  std::to_string((*iter)->Stamp().toSec()) +
                                  "_reference.pcd",
                              cloud_ref_world);
    pcl::io::savePCDFileASCII("/home/nick/tmp/" +
                                  std::to_string((*iter)->Stamp().toSec()) +
                                  "_current_initial.pcd",
                              cloud_cur_initial_world);
    pcl::io::savePCDFileASCII("/home/nick/tmp/" +
                                  std::to_string((*iter)->Stamp().toSec()) +
                                  "_current_aligned.pcd",
                              cloud_cur_aligned_world);
    ////////////////
    */

    // Create a transaction object
    auto transaction = fuse_core::Transaction::make_shared();
    transaction->stamp(msg->header.stamp);
    beam_common::processRelativePoseWithCovariance(
        name(), (*iter)->Position(), (*iter)->Orientation(),
        current_scan_pose->Position(), current_scan_pose->Orientation(),
        T_CLOUDREF_CLOUDCURRENT, covariance, *transaction, true, counter == 1);

    // Send the transaction object to the plugin's parent
    ROS_DEBUG("Sending transaction");
    sendTransaction(transaction);
  }

  // add cloud to reference cloud list and remove last
  if (reference_clouds_.size() == params_.num_neighbors) {
    reference_clouds_.pop_back();
  }
  reference_clouds_.push_front(current_scan_pose);
}

bool ScanMatcher3D::PassedThreshold(const Eigen::Matrix4d& T_measured, const Eigen::Matrix4d& T_estimated){
  Eigen::Matrix3d R1 = T_measured.block(0,0,3,3);
  Eigen::Matrix3d R2 = T_estimated.block(0,0,3,3);

  double t_error = (T_measured.block(0,3,3,1) - T_estimated.block(0,3,3,1)).norm();
  double r_error = std::abs(Eigen::AngleAxis<double>(R1).angle() - Eigen::AngleAxis<double>(R2).angle());

  if(t_error > params_.outlier_threshold_t || r_error > params_.outlier_threshold_r){
    return false;
  }
  return true;
}

void ScanMatcher3D::onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph_msg) {
  // update scan poses
  for (auto iter = reference_clouds_.begin(); iter != reference_clouds_.end();
       iter++) {
    (*iter)->Update(graph_msg);
  }

  // if we aren't saving scans, then no need to worry above active scans list
  if (params_.scan_output_directory.empty()) { return; }

  auto variables = GetPositionVariables(graph_msg);
  for (auto iter = active_clouds_.begin(); iter != active_clouds_.end();
       iter++) {
    // if found, do nothing
    if (variables.find(iter->first) != variables.end()) { continue; }

    // if scan has never been updated, then it is probably just not yet in the
    // window so do nothing
    if (iter->second->Updates() == 0) { continue; }

    // Othewise, it has probably been marginalized out, so save and remove from
    // active list
    iter->second->Save(params_.scan_output_directory);
    active_clouds_.erase(iter->first);
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
  PointCloudPtr cloud2_transformed = boost::make_shared<PointCloud>();
  pcl::transformPointCloud(*cloud2, *cloud2_transformed,
                           Eigen::Affine3d(T_CLOUD1EST_CLOUD2));

  // match clouds
  matcher_->Setup(cloud1, cloud2_transformed);
  matcher_->Match();
  matcher_->EstimateInfo();

  Eigen::Matrix4d T_CLOUD1_CLOUD1EST = matcher_->GetResult().inverse().matrix();
  T_CLOUD1_CLOUD2 = T_CLOUD1_CLOUD1EST * T_CLOUD1EST_CLOUD2;
  covariance = matcher_->GetInfo();
}

std::unordered_set<std::string> ScanMatcher3D::GetPositionVariables(
    fuse_core::Graph::ConstSharedPtr graph_msg) {
  std::unordered_set<std::string> variables_set;
  auto variables = graph_msg->getVariables();
  for (auto i = variables.begin(); i != variables.end(); i++) {
    if (i->type() == "fuse_variables::Position3DStamped") {
      std::string uuid = boost::to_string(i->uuid());
      variables_set.emplace(uuid);
    }
  }
  return variables_set;
}

}} // namespace beam_models::frame_to_frame
