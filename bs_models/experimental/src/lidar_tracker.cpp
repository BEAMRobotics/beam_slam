#include "lidar_tracker.h"

#include <boost/filesystem.hpp>
#include <fuse_core/transaction.h>
#include <pluginlib/class_list_macros.h>

#include <beam_utils/filesystem.h>

#include <bs_common/bs_msgs.h>
#include <bs_models/frame_initializers/frame_initializer.h>
#include <bs_models/scan_registration/multi_scan_registration.h>
#include <bs_models/scan_registration/scan_to_map_registration.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(bs_models::LidarTracker, fuse_core::SensorModel)

namespace bs_models {

using namespace scan_registration;
using namespace beam_matching;

LidarTracker::LidarTracker()
    : fuse_core::AsyncSensorModel(1),
      device_id_(fuse_core::uuid::NIL),
      throttled_callback_(
          std::bind(&LidarTracker::process, this, std::placeholders::_1)) {}

void LidarTracker::onInit() {
  /**
   * @brief THIS CLASS NEEDS TO BE REDONE
   *
   * This file is just a copy of LidarOdometry before we removed the global
   * registration stuff. We need to remove all local registration stuff, then
   * test
   *
   */
  throw std::runtime_error{"THIS CLASS HAS NOT BEEN IMPLEMENTED YET!"};
  params_.loadFromROS(private_node_handle_);

  // setup reloc related params
  active_submap_.SetPublishUpdates(params_.publish_active_submap);
  reloc_request_period_ = ros::Duration(params_.reloc_request_period);

  // init frame initializer
  if (!params_.frame_initializer_config.empty()) {
    frame_initializer_ = std::make_unique<bs_models::FrameInitializer>(
        params_.frame_initializer_config);
  }

  // get filter params
  nlohmann::json J;
  if (!params_.input_filters_config.empty()) {
    std::string filepath = beam::CombinePaths(
        bs_common::GetBeamSlamConfigPath(), params_.input_filters_config);

    ROS_INFO("Reading input filter params from %s", filepath.c_str());
    if (!beam::ReadJson(filepath, J)) {
      ROS_ERROR("Cannot read input filters json, not using any filters.");
    } else {
      nlohmann::json J_filters;
      bool json_valid{true};
      try {
        J_filters = J["filters"];
      } catch (...) {
        ROS_ERROR(
            "Missing 'filters' param in input filters config file. Not using "
            "filters.");
        std::cout << "Json Dump: " << J.dump() << "\n";
        json_valid = false;
      }
      if (json_valid) {
        input_filter_params_ =
            beam_filtering::LoadFilterParamsVector(J_filters);
        ROS_INFO("Loaded %zu input filters", input_filter_params_.size());
      }
    }
  }

  // if outputting scans, clear folders
  if (params_.scan_output_directory.empty()) {
    params_.save_marginalized_scans = false;
    params_.save_graph_updates = false;
    params_.save_scan_registration_results = false;
  } else {
    if (!boost::filesystem::is_directory(params_.scan_output_directory)) {
      boost::filesystem::create_directory(params_.scan_output_directory);
    }

    if (params_.save_graph_updates) {
      graph_updates_path_ =
          beam::CombinePaths(params_.scan_output_directory, "graph_updates");
      if (boost::filesystem::is_directory(graph_updates_path_)) {
        boost::filesystem::remove_all(graph_updates_path_);
      }
      boost::filesystem::create_directory(graph_updates_path_);
    }

    if (params_.save_scan_registration_results) {
      registration_results_path_ = beam::CombinePaths(
          params_.scan_output_directory, "registration_results");
      if (boost::filesystem::is_directory(registration_results_path_)) {
        boost::filesystem::remove_all(registration_results_path_);
      }
      boost::filesystem::create_directory(registration_results_path_);
    }

    if (params_.save_marginalized_scans) {
      marginalized_scans_path_ = beam::CombinePaths(
          params_.scan_output_directory, "marginalized_scans");
      if (boost::filesystem::is_directory(marginalized_scans_path_)) {
        boost::filesystem::remove_all(marginalized_scans_path_);
      }
      boost::filesystem::create_directory(marginalized_scans_path_);
    }
  }
}

void LidarTracker::onStart() {
  subscriber_ = private_node_handle_.subscribe<sensor_msgs::PointCloud2>(
      ros::names::resolve(params_.input_topic), 10,
      &ThrottledCallback::callback, &throttled_callback_,
      ros::TransportHints().tcpNoDelay(false));

  if (params_.output_loam_points || params_.output_lidar_points) {
    results_publisher_ = private_node_handle_.advertise<SlamChunkMsg>(
        "/local_mapper/slam_results", 100);
  }

  if (params_.reloc_request_period != 0) {
    reloc_request_publisher_ = private_node_handle_.advertise<RelocRequestMsg>(
        "/local_mapper/reloc_request", 10);
  }

  if (params_.publish_registration_results) {
    registration_publisher_init_ =
        private_node_handle_.advertise<sensor_msgs::PointCloud2>(
            "registration/initial", 10);
    registration_publisher_aligned_lm_ =
        private_node_handle_.advertise<sensor_msgs::PointCloud2>(
            "registration/aligned_lm", 10);
    registration_publisher_aligned_gm_ =
        private_node_handle_.advertise<sensor_msgs::PointCloud2>(
            "registration/aligned_gm", 10);
  }

  // odometry publishers
  odom_publisher_smooth_ =
      private_node_handle_.advertise<nav_msgs::Odometry>("odom/smooth", 100);
  odom_publisher_global_ =
      private_node_handle_.advertise<nav_msgs::Odometry>("odom/global", 100);
  odom_publisher_marginalized_ =
      private_node_handle_.advertise<nav_msgs::Odometry>("odom/marginalized",
                                                         100);
}

void LidarTracker::onStop() {
  // if output set, save scans before stopping
  ROS_INFO("LidarTracker stopped, processing remaining scans in window.");
  for (auto iter = active_clouds_.begin(); iter != active_clouds_.end();
       iter++) {
    PublishMarginalizedScanPose(*iter);
    if (params_.save_marginalized_scans) {
      (*iter)->SaveCloud(marginalized_scans_path_);
    }
  }

  active_clouds_.clear();
  subscriber_.shutdown();
}

fuse_core::Transaction::SharedPtr LidarTracker::GenerateTransaction(
    const sensor_msgs::PointCloud2::ConstPtr& msg) {
  ROS_DEBUG("Received incoming scan");

  // ensure monotonically increasing data
  if (msg->header.stamp <= last_scan_pose_time_) {
    ROS_WARN(
        "detected non-monotonically increasing lidar stamp, skipping scan");
    return nullptr;
  }
  Eigen::Matrix4d T_WORLD_BASELINKINIT;
  bool init_successful{true};
  std::string error_msg;

  if (frame_initializer_ == nullptr) {
    T_WORLD_BASELINKINIT = T_WORLD_BASELINKLAST_;
  } else if (use_frame_init_relative_) {
    Eigen::Matrix4d T_BASELINKLAST_BASELINKCURRENT;
    init_successful = frame_initializer_->GetRelativePose(
        T_BASELINKLAST_BASELINKCURRENT, last_scan_pose_time_, msg->header.stamp,
        error_msg);
    T_WORLD_BASELINKINIT =
        T_WORLD_BASELINKLAST_ * T_BASELINKLAST_BASELINKCURRENT;
  } else {
    init_successful = frame_initializer_->GetPose(
        T_WORLD_BASELINKINIT, msg->header.stamp,
        extrinsics_.GetBaselinkFrameId(), error_msg);
  }
  if (!init_successful) {
    ROS_DEBUG("Could not initialize frame, skipping scan. Reason: %s",
              error_msg.c_str());
    return nullptr;
  }
  Eigen::Matrix4d T_BASELINK_LIDAR;
  if (!extrinsics_.GetT_BASELINK_LIDAR(T_BASELINK_LIDAR, msg->header.stamp)) {
    ROS_ERROR(
        "Cannot get transform from lidar to baselink for stamp: %.8f. Skipping "
        "scan.",
        msg->header.stamp.toSec());
    return nullptr;
  }

  std::shared_ptr<ScanPose> current_scan_pose;
  if (params_.lidar_type == LidarType::VELODYNE) {
    pcl::PointCloud<PointXYZIRT> cloud_current_unfiltered;
    beam::ROSToPCL(cloud_current_unfiltered, *msg);
    pcl::PointCloud<PointXYZIRT> cloud_filtered =
        beam_filtering::FilterPointCloud<PointXYZIRT>(cloud_current_unfiltered,
                                                      input_filter_params_);
    current_scan_pose = std::make_shared<ScanPose>(
        cloud_filtered, msg->header.stamp, T_WORLD_BASELINKINIT,
        T_BASELINK_LIDAR, feature_extractor_);
  } else if (params_.lidar_type == LidarType::OUSTER) {
    pcl::PointCloud<PointXYZITRRNR> cloud_current_unfiltered;
    beam::ROSToPCL(cloud_current_unfiltered, *msg);
    pcl::PointCloud<PointXYZITRRNR> cloud_filtered =
        beam_filtering::FilterPointCloud(cloud_current_unfiltered,
                                         input_filter_params_);
    current_scan_pose = std::make_shared<ScanPose>(
        cloud_filtered, msg->header.stamp, T_WORLD_BASELINKINIT,
        T_BASELINK_LIDAR, feature_extractor_);
  } else {
    ROS_ERROR(
        "Invalid lidar type param. Lidar type may not be implemented yet.");
  }

  Eigen::Matrix4d T_WORLD_BASELINKCURRENT;
  fuse_core::Transaction::SharedPtr gm_transaction;
  if (global_matching_ || global_loam_matching_) {
    gm_transaction =
        RegisterScanToGlobalMap(*current_scan_pose, T_WORLD_BASELINKCURRENT);
  }

  fuse_core::Transaction::SharedPtr lm_transaction;
  if (local_scan_registration_) {
    lm_transaction =
        local_scan_registration_->RegisterNewScan(*current_scan_pose)
            .GetTransaction();

    Eigen::Matrix4d T_WORLD_LIDAR;
    local_scan_registration_->GetMap().GetScanPose(current_scan_pose->Stamp(),
                                                   T_WORLD_LIDAR);
    T_WORLD_BASELINKCURRENT = T_WORLD_LIDAR * T_BASELINK_LIDAR;
  }

  if (lm_transaction == nullptr && gm_transaction == nullptr) {
    ROS_WARN("No transaction generated, skipping scan.");
    return nullptr;
  }

  // publish global odom
  nav_msgs::Odometry odom_msg_global;
  bs_common::EigenTransformToOdometryMsg(
      T_WORLD_BASELINKCURRENT, current_scan_pose->Stamp(),
      odom_publisher_global_counter_, extrinsics_.GetWorldFrameId(),
      extrinsics_.GetBaselinkFrameId(), odom_msg_global);
  odom_publisher_global_.publish(odom_msg_global);
  odom_publisher_global_counter_++;
  PublishTfTransform(T_WORLD_BASELINKCURRENT, "lidar_world",
                     extrinsics_.GetBaselinkFrameId(),
                     current_scan_pose->Stamp());

  // publish smooth odom
  nav_msgs::Odometry odom_msg_smooth;
  Eigen::Matrix4d T_BASELINKLAST_BASELINKCURRENT =
      beam::InvertTransform(T_WORLD_BASELINKLAST_) * T_WORLD_BASELINKCURRENT;
  Eigen::Matrix4d T_WORLD_BASELINKSMOOTH =
      T_WORLD_BASELINKLAST_ * T_BASELINKLAST_BASELINKCURRENT;
  bs_common::EigenTransformToOdometryMsg(
      T_WORLD_BASELINKSMOOTH, current_scan_pose->Stamp(),
      odom_publisher_smooth_counter_, extrinsics_.GetWorldFrameId(),
      extrinsics_.GetBaselinkFrameId(), odom_msg_smooth);
  odom_publisher_smooth_.publish(odom_msg_smooth);
  odom_publisher_smooth_counter_++;
  PublishTfTransform(T_WORLD_BASELINKSMOOTH, "lidar_world_smooth",
                     extrinsics_.GetBaselinkFrameId(),
                     current_scan_pose->Stamp());

  // add priors from initializer
  fuse_core::Transaction::SharedPtr prior_transaction;
  if (params_.prior_information_weight != 0) {
    auto p = fuse_variables::Position3DStamped::make_shared(
        current_scan_pose->Position());
    auto o = fuse_variables::Orientation3DStamped::make_shared(
        current_scan_pose->Orientation());
    prior_transaction = fuse_core::Transaction::make_shared();
    prior_transaction->stamp(current_scan_pose->Stamp());
    prior_transaction->addVariable(p);
    prior_transaction->addVariable(o);

    // add prior
    fuse_core::Vector7d mean;
    mean << p->x(), p->y(), p->z(), o->w(), o->x(), o->y(), o->z();

    auto prior =
        std::make_shared<fuse_constraints::AbsolutePose3DStampedConstraint>(
            "FRAMEINITIALIZERPRIOR", *p, *o, mean, params_.prior_covariance);
    prior_transaction->addConstraint(prior);
  }

  PublishScanRegistrationResults(lm_transaction, gm_transaction,
                                 *current_scan_pose);
  active_clouds_.push_back(current_scan_pose);

  // combine to one transaction and return
  fuse_core::Transaction::SharedPtr transaction =
      fuse_core::Transaction::make_shared();
  if (lm_transaction != nullptr) { transaction->merge(*lm_transaction); }
  if (gm_transaction != nullptr) { transaction->merge(*gm_transaction); }
  if (prior_transaction != nullptr) { transaction->merge(*prior_transaction); }

  // set current measurements to last
  T_WORLD_BASELINKLAST_ = T_WORLD_BASELINKCURRENT;
  last_scan_pose_time_ = current_scan_pose->Stamp();

  return transaction;
}

void LidarTracker::SetupRegistration() {
  // setup local registration
  beam_matching::MatcherType local_matcher_type;
  if (!params_.local_matcher_config.empty()) {
    const auto& reg_filepath = params_.local_registration_config;
    const auto& matcher_filepath = params_.local_matcher_config;
    local_scan_registration_ = ScanRegistrationBase::Create(
        reg_filepath, matcher_filepath, registration_results_path_);

    // setup feature extractor if needed
    local_matcher_type = beam_matching::GetTypeFromConfig(matcher_filepath);
    if (local_matcher_type == beam_matching::MatcherType::LOAM) {
      std::shared_ptr<LoamParams> matcher_params =
          std::make_shared<LoamParams>(matcher_filepath);
      feature_extractor_ =
          std::make_shared<LoamFeatureExtractor>(matcher_params);
    }
  }

  // set registration map to publish
  RegistrationMap& map = RegistrationMap::GetInstance();
  if (params_.publish_local_map) {
    map.SetPublishUpdates(true);
    ROS_INFO("Publishing initial lidar_odometry registration map");
    map.Publish();
  }

  // Get last scan pose to initialize with if registration map isn't empty
  if (!map.Empty()) {
    // get extrinsics
    Eigen::Matrix4d T_BASELINK_LIDAR;
    if (!extrinsics_.GetT_BASELINK_LIDAR(T_BASELINK_LIDAR)) {
      ROS_ERROR(
          "Cannot lookup transform from lidar to baselink, not sending reloc "
          "request.");
      throw std::runtime_error{"cannot lookup transform"};
    }

    // get scan pose
    Eigen::Matrix4d T_MAP_SCAN;
    ros::Time last_stamp;
    last_scan_pose_time_ = map.GetLastCloudPoseStamp();
    map.GetScanPose(last_scan_pose_time_, T_MAP_SCAN);

    T_WORLD_BASELINKLAST_ =
        T_MAP_SCAN * beam::InvertTransform(T_BASELINK_LIDAR);
  }

  // Setup global registration matcher
  if (!params_.global_matcher_config.empty()) {
    const auto& filepath = params_.global_matcher_config;
    auto matcher_type = beam_matching::GetTypeFromConfig(filepath);

    if (matcher_type == beam_matching::MatcherType::LOAM) {
      global_loam_matching_ =
          std::make_unique<LoamMatcher>(LoamParams(filepath));
    } else if (matcher_type == beam_matching::MatcherType::ICP) {
      global_matching_ =
          std::make_unique<IcpMatcher>(IcpMatcher::Params(filepath));
    } else if (matcher_type == beam_matching::MatcherType::GICP) {
      global_matching_ =
          std::make_unique<GicpMatcher>(GicpMatcher::Params(filepath));
    } else if (matcher_type == beam_matching::MatcherType::NDT) {
      global_matching_ =
          std::make_unique<NdtMatcher>(NdtMatcher::Params(filepath));
    } else {
      ROS_ERROR(
          "Invalid global matcher type. Not running global map registration");
    }
  }

  local_scan_registration_->SetInformationWeight(
      params_.lidar_information_weight);
}

fuse_core::Transaction::SharedPtr
    LidarTracker::RegisterScanToGlobalMap(const ScanPose& scan_pose,
                                          Eigen::Matrix4d& T_MAP_BASELINK) {
  Eigen::Matrix4d T_MAPEST_SCAN = scan_pose.T_REFFRAME_LIDAR();
  Eigen::Matrix4d T_MAPEST_MAP;

  if (global_loam_matching_ != nullptr) {
    LoamPointCloudPtr scan_in_map_frame =
        std::make_shared<LoamPointCloud>(scan_pose.LoamCloud());
    scan_in_map_frame->TransformPointCloud(T_MAPEST_SCAN);

    LoamPointCloudPtr current_map = active_submap_.GetLoamMapPtr();
    if (current_map->Empty()) {
      ROS_WARN_THROTTLE(
          5, "active submap empty, make sure you have a global mapper running");
      return nullptr;
    }

    global_loam_matching_->SetRef(current_map);
    global_loam_matching_->SetTarget(scan_in_map_frame);

    if (!global_loam_matching_->Match()) { return nullptr; }

    T_MAPEST_MAP = global_loam_matching_->GetResult().matrix();
  } else {
    PointCloudPtr scan_in_map_frame = std::make_shared<PointCloud>();
    pcl::transformPointCloud(scan_pose.Cloud(), *scan_in_map_frame,
                             T_MAPEST_SCAN);

    PointCloudPtr current_map = active_submap_.GetLidarMap();

    if (current_map->empty()) {
      ROS_WARN_THROTTLE(
          5, "active submap empty, make sure you have a global mapper running");
      return nullptr;
    }

    global_matching_->SetRef(current_map);
    global_matching_->SetTarget(scan_in_map_frame);

    if (!global_matching_->Match()) { return nullptr; }

    T_MAPEST_MAP = global_matching_->GetResult().matrix();
  }

  // check against threshold. Use threshold params from local matcher
  // NOTE WE HAVE CHANGED TO RegistrationValidation class
  if (!local_scan_registration_->PassedRegThreshold(T_MAPEST_MAP)) {
    ROS_WARN(
        "Failed global scan matcher transform threshold check for stamp %d.%d. "
        "Skipping measurement.",
        scan_pose.Stamp().sec, scan_pose.Stamp().nsec);
    return nullptr;
  }

  // get extrinsics
  Eigen::Matrix4d T_LIDAR_BASELINK;
  if (!extrinsics_.GetT_LIDAR_BASELINK(T_LIDAR_BASELINK)) {
    ROS_WARN(
        "Cannot get transform from baselink to lidar. Not adding global map "
        "constraint.");
    return nullptr;
  }

  // get absolute pose
  T_MAP_BASELINK =
      beam::InvertTransform(T_MAPEST_MAP) * T_MAPEST_SCAN * T_LIDAR_BASELINK;
  Eigen::Matrix3d R = T_MAP_BASELINK.block(0, 0, 3, 3);
  Eigen::Quaterniond q(R);
  fuse_core::Vector7d mean;
  mean << T_MAP_BASELINK(0, 3), T_MAP_BASELINK(1, 3), T_MAP_BASELINK(2, 3),
      q.w(), q.x(), q.y(), q.z();

  // create transaction and add variables and prior
  auto p = fuse_variables::Position3DStamped::make_shared(scan_pose.Position());
  auto o = fuse_variables::Orientation3DStamped::make_shared(
      scan_pose.Orientation());

  fuse_core::Transaction::SharedPtr transaction =
      fuse_core::Transaction::make_shared();
  transaction->stamp(scan_pose.Stamp());
  transaction->addVariable(p);
  transaction->addVariable(o);
  double cov_weight =
      1 / (params_.lidar_information_weight * params_.lidar_information_weight);
  auto prior =
      std::make_shared<fuse_constraints::AbsolutePose3DStampedConstraint>(
          "GLOBALMAPREGISTRATION", *p, *o, mean,
          cov_weight * global_matching_->GetCovariance());
  transaction->addConstraint(prior);
  return transaction;
}

void LidarTracker::onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph_msg) {
  if (updates_ == 0) {
    ROS_INFO("received first graph update, initializing registration and "
             "starting lidar odometry");
    SetupRegistration();
  }
  updates_++;

  auto i = active_clouds_.begin();
  while (i != active_clouds_.end()) {
    std::shared_ptr<ScanPose>& scan_pose = *i;
    bool update_successful = scan_pose->UpdatePose(graph_msg);
    if (update_successful) {
      // update map
      if (update_local_map_on_graph_update_) {
        local_scan_registration_->GetMapMutable().UpdateScan(
            scan_pose->Stamp(), scan_pose->T_REFFRAME_LIDAR());
      }

      // send reloc request if it is the first time the scan pose is updated,
      // and if the time elapsed since the last reloc request is greater than
      // the min.
      if (scan_pose->Updates() == 1 ||
          scan_pose->Stamp() - last_reloc_request_time_ >=
              reloc_request_period_) {
        SendRelocRequest(scan_pose);
      }

      ++i;
      continue;
    }

    // if scan has never been updated, then it is probably just not yet in the
    // window so do nothing
    if (scan_pose->Updates() == 0) {
      ++i;
      continue;
    }

    // Othewise, it has probably been marginalized out, so output and remove
    // from active list
    PublishMarginalizedScanPose(*i);
    if (params_.save_marginalized_scans) {
      (*i)->SaveCloud(marginalized_scans_path_);
    }
    active_clouds_.erase(i++);
  }

  if (!params_.save_graph_updates) { return; }

  std::string update_time =
      beam::ConvertTimeToDate(std::chrono::system_clock::now());
  std::string curent_path = beam::CombinePaths(
      graph_updates_path_, "U" + std::to_string(updates_) + "_" + update_time);
  boost::filesystem::create_directory(curent_path);
  for (auto iter = active_clouds_.begin(); iter != active_clouds_.end();
       iter++) {
    (*iter)->SaveCloud(curent_path);
  }
}

void LidarTracker::process(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  if (updates_ == 0) {
    ROS_INFO_THROTTLE(
        1, "lidar odometry not yet initialized, waiting on first graph "
           "update before beginning");
    return;
  }

  fuse_core::Transaction::SharedPtr new_transaction = GenerateTransaction(msg);
  if (new_transaction != nullptr) {
    ROS_DEBUG("Sending transaction.");
    try {
      sendTransaction(new_transaction);
    } catch (const std::exception& e) {
      ROS_WARN("Cannot send transaction. Error: %s", e.what());
    }
  }
}

void LidarTracker::SendRelocRequest(
    const std::shared_ptr<ScanPose>& scan_pose) {
  // Get extrinsics
  Eigen::Matrix4d T_BASELINK_LIDAR;
  if (!extrinsics_.GetT_BASELINK_LIDAR(T_BASELINK_LIDAR)) {
    ROS_ERROR(
        "Cannot lookup transform from lidar to baselink, not sending reloc "
        "request.");
    return;
  }

  // get clouds in baselink frame
  PointCloud cloud_in_baselink_frame;
  pcl::transformPointCloud(scan_pose->Cloud(), cloud_in_baselink_frame,
                           T_BASELINK_LIDAR);
  beam_matching::LoamPointCloud loam_cloud_in_baselink_frame(
      scan_pose->LoamCloud(), T_BASELINK_LIDAR);

  // create message and publish
  //! why is reloc frame id = baselink, while slam chunk frame id = lidar? line
  //! 624
  RelocRequestMsg msg;
  static uint64_t seq = 0;
  geometry_msgs::PoseStamped pose_stamped;
  bs_common::EigenTransformToPoseStamped(
      scan_pose->T_REFFRAME_BASELINK(), scan_pose->Stamp(), seq++,
      extrinsics_.GetBaselinkFrameId(), pose_stamped);
  msg.T_WORLD_BASELINK = pose_stamped;
  msg.lidar_measurement.frame_id = extrinsics_.GetBaselinkFrameId();
  msg.lidar_measurement.lidar_points =
      beam::PCLToROSVector(cloud_in_baselink_frame);
  msg.lidar_measurement.lidar_edges_strong =
      beam::PCLToROSVector(loam_cloud_in_baselink_frame.edges.strong.cloud);
  msg.lidar_measurement.lidar_edges_weak =
      beam::PCLToROSVector(loam_cloud_in_baselink_frame.edges.weak.cloud);
  msg.lidar_measurement.lidar_surfaces_strong =
      beam::PCLToROSVector(loam_cloud_in_baselink_frame.surfaces.strong.cloud);
  msg.lidar_measurement.lidar_surfaces_weak =
      beam::PCLToROSVector(loam_cloud_in_baselink_frame.surfaces.strong.cloud);

  reloc_request_publisher_.publish(msg);

  last_reloc_request_time_ = scan_pose->Stamp();
}

void LidarTracker::PublishMarginalizedScanPose(
    const std::shared_ptr<ScanPose>& scan_pose) {
  nav_msgs::Odometry odom_msg;
  bs_common::EigenTransformToOdometryMsg(
      scan_pose->T_REFFRAME_BASELINK(), scan_pose->Stamp(),
      odom_publisher_marginalized_counter_, "lidar_world_marginalized",
      extrinsics_.GetBaselinkFrameId(), odom_msg);
  odom_publisher_marginalized_.publish(odom_msg);
  odom_publisher_marginalized_counter_++;

  if (!params_.output_loam_points && params_.output_lidar_points) { return; }

  // output to global mapper
  SlamChunkMsg slam_chunk_msg;
  static uint64_t seq = 0;
  geometry_msgs::PoseStamped pose_stamped;
  bs_common::EigenTransformToPoseStamped(
      scan_pose->T_REFFRAME_BASELINK(), scan_pose->Stamp(), seq++,
      extrinsics_.GetLidarFrameId(), pose_stamped);
  slam_chunk_msg.T_WORLD_BASELINK = pose_stamped;
  slam_chunk_msg.lidar_measurement.frame_id = extrinsics_.GetLidarFrameId();

  if (params_.output_lidar_points) {
    // add regular points
    const PointCloud& cloud = scan_pose->Cloud();
    for (const auto& p : cloud) {
      geometry_msgs::Vector3 point;
      point.x = p.x;
      point.y = p.y;
      point.z = p.z;
      slam_chunk_msg.lidar_measurement.lidar_points.push_back(point);
    }
  }

  // if loam cloud not to be outputted, or empty, publish current msg
  const beam_matching::LoamPointCloud& loam_cloud = scan_pose->LoamCloud();
  if (params_.output_loam_points) {
    // add strong edges
    for (const auto& p : loam_cloud.edges.strong.cloud) {
      geometry_msgs::Vector3 point;
      point.x = p.x;
      point.y = p.y;
      point.z = p.z;
      slam_chunk_msg.lidar_measurement.lidar_edges_strong.push_back(point);
    }

    // add weak edges
    for (const auto& p : loam_cloud.edges.weak.cloud) {
      geometry_msgs::Vector3 point;
      point.x = p.x;
      point.y = p.y;
      point.z = p.z;
      slam_chunk_msg.lidar_measurement.lidar_edges_weak.push_back(point);
    }

    // add strong surfaces
    for (const auto& p : loam_cloud.surfaces.strong.cloud) {
      geometry_msgs::Vector3 point;
      point.x = p.x;
      point.y = p.y;
      point.z = p.z;
      slam_chunk_msg.lidar_measurement.lidar_surfaces_strong.push_back(point);
    }

    // add weak surfaces
    for (const auto& p : loam_cloud.surfaces.weak.cloud) {
      geometry_msgs::Vector3 point;
      point.x = p.x;
      point.y = p.y;
      point.z = p.z;
      slam_chunk_msg.lidar_measurement.lidar_surfaces_weak.push_back(point);
    }
  }

  ROS_DEBUG("Publishing slam chunk msg");
  results_publisher_.publish(slam_chunk_msg);
}

void LidarTracker::PublishScanRegistrationResults(
    const fuse_core::Transaction::SharedPtr& transaction_lm,
    const fuse_core::Transaction::SharedPtr& transaction_gm,
    const ScanPose& scan_pose) {
  if (!params_.publish_registration_results) { return; }
  if (transaction_lm == nullptr && transaction_gm == nullptr) { return; }

  const PointCloud& scan_in_lidar_frame = scan_pose.Cloud();
  Eigen::Matrix4d T_WORLD_BASELINKINIT = scan_pose.T_REFFRAME_BASELINK();
  Eigen::Matrix4d T_BASELINK_LIDAR = scan_pose.T_BASELINK_LIDAR();

  const auto& sc_orientation = scan_pose.Orientation();
  const auto& sc_position = scan_pose.Position();
  fuse_constraints::AbsolutePose3DStampedConstraint dummy_abs_const;

  // get pose of baselink as measured by the GM from transaction_gm prior
  Eigen::Matrix4d T_WORLD_BASELINKREFGM = Eigen::Matrix4d::Identity();
  bool publish_gm_results{false};
  if (transaction_gm != nullptr) {
    auto added_constraints = transaction_gm->addedConstraints();
    int pose_constraints_found = 0;
    for (auto iter = added_constraints.begin(); iter != added_constraints.end();
         iter++) {
      if (iter->type() == dummy_abs_const.type()) {
        pose_constraints_found++;
        auto constraint = dynamic_cast<
            const fuse_constraints::AbsolutePose3DStampedConstraint&>(*iter);
        const fuse_core::Vector7d& mean = constraint.mean();

        Eigen::Vector3d t_WORLD_BASELINKREFGM =
            Eigen::Vector3d(mean[0], mean[1], mean[2]);
        Eigen::Quaterniond q(mean[3], mean[4], mean[5], mean[6]);
        Eigen::Matrix3d R_WORLD_BASELINKREFGM(q);

        T_WORLD_BASELINKREFGM.block(0, 0, 3, 3) = R_WORLD_BASELINKREFGM;
        T_WORLD_BASELINKREFGM.block(0, 3, 3, 1) = t_WORLD_BASELINKREFGM;
      }
    }

    // check we have the right amount of constraints
    if (pose_constraints_found == 1) {
      publish_gm_results = true;
    } else {
      publish_gm_results = false;
    }
  }

  // get pose of baselink as measured by the LM from transaction_lm binary
  // constraint
  fuse_constraints::RelativePose3DStampedConstraint dummy_rel_pose_const;
  Eigen::Matrix4d T_WORLD_LIDARREFLM = Eigen::Matrix4d::Identity();
  bool publish_lm_results{false};
  if (transaction_lm != nullptr) {
    std::vector<Eigen::Matrix4d, beam::AlignMat4d> Ts_WORLD_LIDARREFLM;
    auto added_constraints = transaction_lm->addedConstraints();
    for (auto iter = added_constraints.begin(); iter != added_constraints.end();
         iter++) {
      if (iter->type() == dummy_rel_pose_const.type()) {
        Eigen::Matrix4d T_REFFRAME_BASELINKREFLM = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d T_WORLD_REFFRAME = Eigen::Matrix4d::Identity();

        auto constraint = dynamic_cast<
            const fuse_constraints::RelativePose3DStampedConstraint&>(*iter);
        const fuse_core::Vector7d& mean = constraint.delta();
        const std::vector<fuse_core::UUID>& variables = constraint.variables();

        // build transform between two poses
        Eigen::Vector3d t_REF_SCAN = Eigen::Vector3d(mean[0], mean[1], mean[2]);
        Eigen::Quaterniond q(mean[3], mean[4], mean[5], mean[6]);
        Eigen::Matrix3d R_REF_SCAN(q);
        Eigen::Matrix4d T_REF_SCAN = Eigen::Matrix4d::Identity();
        T_REF_SCAN.block(0, 0, 3, 3) = R_REF_SCAN;
        T_REF_SCAN.block(0, 3, 3, 1) = t_REF_SCAN;

        // get pose of reference
        const RegistrationMap& map = local_scan_registration_->GetMap();
        ros::Time ref_stamp;
        if (!map.GetUUIDStamp(variables.at(0), ref_stamp)) {
          ROS_WARN("UUID not found in registration map, not publishing scan "
                   "registration result.");
          continue;
        }
        Eigen::Matrix4d T_WORLD_REF;
        map.GetScanPose(ref_stamp, T_WORLD_REF);
        Ts_WORLD_LIDARREFLM.push_back(T_WORLD_REF *
                                      beam::InvertTransform(T_REF_SCAN));
      }
    }
    if (Ts_WORLD_LIDARREFLM.size() > 0) {
      T_WORLD_LIDARREFLM = beam::AverageTransforms(Ts_WORLD_LIDARREFLM);
      publish_lm_results = true;
    } else {
      publish_lm_results = false;
    }
  }

  // convert to lidar poses
  Eigen::Matrix4d T_WORLD_LIDARINIT = T_WORLD_BASELINKINIT * T_BASELINK_LIDAR;
  Eigen::Matrix4d T_WORLD_LIDARREFGM = T_WORLD_BASELINKREFGM * T_BASELINK_LIDAR;

  PointCloud scan_in_world_init_frame;
  pcl::transformPointCloud(scan_in_lidar_frame, scan_in_world_init_frame,
                           T_WORLD_LIDARINIT);
  sensor_msgs::PointCloud2 msg_init = beam::PCLToROS<pcl::PointXYZ>(
      scan_in_world_init_frame, scan_pose.Stamp(),
      extrinsics_.GetWorldFrameId(), published_registration_results_);
  registration_publisher_init_.publish(msg_init);

  if (publish_lm_results) {
    PointCloud scan_in_world_lm_frame;

    pcl::transformPointCloud(scan_in_lidar_frame, scan_in_world_lm_frame,
                             T_WORLD_LIDARREFLM);
    sensor_msgs::PointCloud2 msg_align_lm = beam::PCLToROS<pcl::PointXYZ>(
        scan_in_world_lm_frame, scan_pose.Stamp(),
        extrinsics_.GetWorldFrameId(), published_registration_results_);

    registration_publisher_aligned_lm_.publish(msg_align_lm);
  }
  if (publish_gm_results) {
    PointCloud scan_in_world_gm_frame;
    pcl::transformPointCloud(scan_in_lidar_frame, scan_in_world_gm_frame,
                             T_WORLD_LIDARREFGM);
    sensor_msgs::PointCloud2 msg_align_gm = beam::PCLToROS<pcl::PointXYZ>(
        scan_in_world_gm_frame, scan_pose.Stamp(),
        extrinsics_.GetWorldFrameId(), published_registration_results_);
    registration_publisher_aligned_gm_.publish(msg_align_gm);
  }

  published_registration_results_++;
}

void LidarTracker::PublishTfTransform(const Eigen::Matrix4d& T_CHILD_PARENT,
                                      const std::string& child_frame,
                                      const std::string& parent_frame,
                                      const ros::Time& time) {
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(T_CHILD_PARENT(0, 3), T_CHILD_PARENT(1, 3),
                                  T_CHILD_PARENT(2, 3)));

  Eigen::Matrix3d R = T_CHILD_PARENT.block(0, 0, 3, 3);
  Eigen::Quaterniond q(R);
  tf::Quaternion q_tf(q.x(), q.y(), q.z(), q.w());
  transform.setRotation(q_tf);
  tf_broadcaster_.sendTransform(
      tf::StampedTransform(transform, time, parent_frame, child_frame));
}

} // namespace bs_models
