#include <bs_models/lidar_odometry.h>

#include <boost/filesystem.hpp>
#include <fuse_core/transaction.h>
#include <pluginlib/class_list_macros.h>

#include <beam_utils/filesystem.h>

#include <bs_common/bs_msgs.h>
#include <bs_models/frame_initializers/frame_initializers.h>
#include <bs_models/scan_registration/multi_scan_registration.h>
#include <bs_models/scan_registration/scan_to_map_registration.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(bs_models::LidarOdometry, fuse_core::SensorModel)

namespace bs_models {

using namespace scan_registration;

LidarOdometry::LidarOdometry()
    : fuse_core::AsyncSensorModel(1),
      device_id_(fuse_core::uuid::NIL),
      throttled_callback_(
          std::bind(&LidarOdometry::process, this, std::placeholders::_1)) {}

void LidarOdometry::onInit() {
  params_.loadFromROS(private_node_handle_);

  // setup reloc related params
  active_submap_.SetPublishUpdates(params_.publish_active_submap);
  reloc_request_period_ = ros::Duration(params_.reloc_request_period);

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
  } else if (params_.frame_initializer_type == "TRANSFORM") {
    frame_initializer_ =
        std::make_unique<frame_initializers::TransformFrameInitializer>(
            params_.frame_initializer_info, 100, 30,
            params_.sensor_frame_id_override);
  } else {
    const std::string error =
        "frame_initializer_type invalid. Options: ODOMETRY, POSEFILE, TRANSFORM";
    ROS_FATAL_STREAM(error);
    throw std::runtime_error(error);
  }

  // init scan registration
  SetupRegistration();

  // get filter params
  nlohmann::json J;
  std::string filepath = params_.input_filters_config_path;
  if (!filepath.empty()) {
    if (filepath == "DEFAULT_PATH") {
      filepath = bs_common::GetBeamSlamConfigPath() +
                 "registration_config/input_filters.json";
    }

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
        ROS_INFO("Loaded %d input filters", input_filter_params_.size());
      }
    }
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

void LidarOdometry::onStart() {
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

  if (params_.publish_local_map) {
    RegistrationMap& map = RegistrationMap::GetInstance();
    map.SetParams(map.MapSize(), true);
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
}

void LidarOdometry::onStop() {
  // if output set, save scans before stopping
  ROS_INFO("LidarOdometry stopped, processing remaining scans in window.");
  for (auto iter = active_clouds_.begin(); iter != active_clouds_.end();
       iter++) {
    PublishMarginalizedScanPose(*iter);
    if (!params_.scan_output_directory.empty()) {
      iter->SaveCloud(params_.scan_output_directory);
    }
  }

  active_clouds_.clear();
  subscriber_.shutdown();
}

fuse_core::Transaction::SharedPtr LidarOdometry::GenerateTransaction(
    const sensor_msgs::PointCloud2::ConstPtr& msg) {
  ROS_DEBUG("Received incoming scan");

  Eigen::Matrix4d T_WORLD_BASELINKCURRENT;
  if (!frame_initializer_->GetEstimatedPose(T_WORLD_BASELINKCURRENT,
                                            msg->header.stamp,
                                            extrinsics_.GetBaselinkFrameId())) {
    ROS_DEBUG("Skipping scan");
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
        cloud_filtered, msg->header.stamp, T_WORLD_BASELINKCURRENT,
        T_BASELINK_LIDAR, feature_extractor_);
  } else if (params_.lidar_type == LidarType::OUSTER) {
    pcl::PointCloud<PointXYZITRRNR> cloud_current_unfiltered;
    beam::ROSToPCL(cloud_current_unfiltered, *msg);
    pcl::PointCloud<PointXYZITRRNR> cloud_filtered =
        beam_filtering::FilterPointCloud(cloud_current_unfiltered,
                                         input_filter_params_);
    current_scan_pose = std::make_shared<ScanPose>(
        cloud_filtered, msg->header.stamp, T_WORLD_BASELINKCURRENT,
        T_BASELINK_LIDAR, feature_extractor_);
  } else {
    BEAM_ERROR(
        "Invalid lidar type param. Lidar type may not be implemented yet.");
  }

  fuse_core::Transaction::SharedPtr gm_transaction;
  if (params_.register_to_gm) {
    gm_transaction = RegisterScanToGlobalMap(*current_scan_pose);
  }

  fuse_core::Transaction::SharedPtr lm_transaction;
  if (params_.register_to_lm) {
    lm_transaction =
        local_scan_registration_->RegisterNewScan(*current_scan_pose)
            .GetTransaction();
  }

  // add priors from initializer
  fuse_core::Transaction::SharedPtr prior_transaction;
  if (params_.use_pose_priors) {
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

  if (lm_transaction == nullptr && gm_transaction == nullptr &&
      prior_transaction == nullptr) {
    ROS_ERROR("No transaction generated, skipping scan.");
    return nullptr;
  }

  PublishScanRegistrationResults(lm_transaction, gm_transaction,
                                 *current_scan_pose);
  active_clouds_.push_back(*current_scan_pose);

  // combine to one transaction and return
  fuse_core::Transaction::SharedPtr transaction =
      fuse_core::Transaction::make_shared();
  if (lm_transaction != nullptr) { transaction->merge(*lm_transaction); }
  if (gm_transaction != nullptr) { transaction->merge(*gm_transaction); }
  if (prior_transaction != nullptr) { transaction->merge(*prior_transaction); }

  return transaction;
}

void LidarOdometry::SetupRegistration() {
  // setup local registration
  if (params_.local_registration_type == "MAPLOAM") {
    std::shared_ptr<LoamParams> matcher_params =
        std::make_shared<LoamParams>(params_.local_matcher_params_path);
    std::unique_ptr<Matcher<LoamPointCloudPtr>> matcher =
        std::make_unique<LoamMatcher>(*matcher_params);
    ScanToMapLoamRegistration::Params params;
    params.LoadFromJson(params_.local_registration_config_path);
    local_scan_registration_ = std::make_unique<ScanToMapLoamRegistration>(
        std::move(matcher), params.GetBaseParams(), params.map_size,
        params.store_full_cloud);
    feature_extractor_ = std::make_shared<LoamFeatureExtractor>(matcher_params);
  } else if (params_.local_registration_type == "MULTIICP") {
    std::unique_ptr<Matcher<PointCloudPtr>> matcher =
        std::make_unique<IcpMatcher>(
            IcpMatcher::Params(params_.local_matcher_params_path));
    MultiScanRegistrationBase::Params params;
    params.LoadFromJson(params_.local_registration_config_path);
    local_scan_registration_ = std::make_unique<MultiScanRegistration>(
        std::move(matcher), params.GetBaseParams(), params.num_neighbors,
        params.lag_duration, params.disable_lidar_map);
  } else if (params_.local_registration_type == "MULTINDT") {
    std::unique_ptr<Matcher<PointCloudPtr>> matcher =
        std::make_unique<NdtMatcher>(
            NdtMatcher::Params(params_.local_matcher_params_path));
    MultiScanRegistrationBase::Params params;
    params.LoadFromJson(params_.local_registration_config_path);
    local_scan_registration_ = std::make_unique<MultiScanRegistration>(
        std::move(matcher), params.GetBaseParams(), params.num_neighbors,
        params.lag_duration, params.disable_lidar_map);
  } else if (params_.local_registration_type == "MULTIGICP") {
    std::unique_ptr<Matcher<PointCloudPtr>> matcher =
        std::make_unique<GicpMatcher>(
            GicpMatcher::Params(params_.local_matcher_params_path));
    MultiScanRegistrationBase::Params params;
    params.LoadFromJson(params_.local_registration_config_path);
    local_scan_registration_ = std::make_unique<MultiScanRegistration>(
        std::move(matcher), params.GetBaseParams(), params.num_neighbors,
        params.lag_duration, params.disable_lidar_map);
  } else if (params_.local_registration_type == "MULTILOAM") {
    std::shared_ptr<LoamParams> matcher_params =
        std::make_shared<LoamParams>(params_.local_matcher_params_path);
    std::unique_ptr<Matcher<LoamPointCloudPtr>> matcher =
        std::make_unique<LoamMatcher>(*matcher_params);
    MultiScanRegistrationBase::Params params;
    params.LoadFromJson(params_.local_registration_config_path);
    local_scan_registration_ = std::make_unique<MultiScanLoamRegistration>(
        std::move(matcher), params.GetBaseParams(), params.num_neighbors,
        params.lag_duration, params.disable_lidar_map);
    feature_extractor_ = std::make_shared<LoamFeatureExtractor>(matcher_params);
  } else {
    BEAM_ERROR(
        "Invalid local_registration_type. Input: {}. Options: MAPLOAM, "
        "MULTILOAM, MULTIICP, MULTIGICP, MULTINDT. Using default: MAPLOAM",
        params_.local_registration_type);
    std::shared_ptr<LoamParams> matcher_params =
        std::make_shared<LoamParams>(params_.local_matcher_params_path);
    std::unique_ptr<Matcher<LoamPointCloudPtr>> matcher =
        std::make_unique<LoamMatcher>(*matcher_params);
    ScanToMapLoamRegistration::Params params;
    params.LoadFromJson(params_.local_registration_config_path);
    local_scan_registration_ = std::make_unique<ScanToMapLoamRegistration>(
        std::move(matcher), params.GetBaseParams(), params.map_size,
        params.store_full_cloud);
    feature_extractor_ = std::make_shared<LoamFeatureExtractor>(matcher_params);
  }
  local_scan_registration_->SetFixedCovariance(
      params_.local_registration_covariance);

  // Setup global registration matcher
  if (params_.global_registration_type == "LOAM") {
    global_loam_matching_ = std::make_unique<LoamMatcher>(
        LoamParams(params_.global_matcher_params_path));
  } else if (params_.global_registration_type == "ICP") {
    global_matching_ = std::make_unique<IcpMatcher>(
        IcpMatcher::Params(params_.global_matcher_params_path));
  } else if (params_.global_registration_type == "GICP") {
    global_matching_ = std::make_unique<GicpMatcher>(
        GicpMatcher::Params(params_.global_matcher_params_path));
  } else if (params_.global_registration_type == "NDT") {
    global_matching_ = std::make_unique<NdtMatcher>(
        NdtMatcher::Params(params_.global_matcher_params_path));
  } else {
    BEAM_ERROR(
        "Invalid global_registration_type. Input: {}. Options: LOAM, ICP, "
        "GICP, NDT. Using default: "
        "LOAM",
        params_.global_registration_type);
    global_loam_matching_ = std::make_unique<LoamMatcher>(
        LoamParams(params_.global_matcher_params_path));
  }
}

fuse_core::Transaction::SharedPtr
    LidarOdometry::RegisterScanToGlobalMap(const ScanPose& scan_pose) {
  Eigen::Matrix4d T_MAPEST_SCAN = scan_pose.T_REFFRAME_LIDAR();
  Eigen::Matrix4d T_MAPEST_MAP;

  if (global_loam_matching_ != nullptr) {
    LoamPointCloudPtr scan_in_map_frame =
        std::make_shared<LoamPointCloud>(scan_pose.LoamCloud());
    scan_in_map_frame->TransformPointCloud(T_MAPEST_SCAN);

    LoamPointCloudPtr current_map = active_submap_.GetLoamMapPtr();

    global_loam_matching_->SetRef(current_map);
    global_loam_matching_->SetTarget(scan_in_map_frame);

    if (!global_loam_matching_->Match()) { return nullptr; }

    T_MAPEST_MAP = global_loam_matching_->GetResult().matrix();
  } else {
    PointCloudPtr scan_in_map_frame = std::make_shared<PointCloud>();
    pcl::transformPointCloud(scan_pose.Cloud(), *scan_in_map_frame,
                             T_MAPEST_SCAN);

    PointCloudPtr current_map = active_submap_.GetLidarMap();

    global_matching_->SetRef(current_map);
    global_matching_->SetTarget(scan_in_map_frame);

    if (!global_matching_->Match()) { return nullptr; }

    T_MAPEST_MAP = global_matching_->GetResult().matrix();
  }

  // check against threshold. Use threshold params from local matcher
  if (!local_scan_registration_->PassedRegThreshold(T_MAPEST_MAP)) {
    BEAM_WARN(
        "Failed global scan matcher transform threshold check for stamp {}.{}. "
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
  Eigen::Matrix4d T_MAP_BASELINK =
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
  auto prior =
      std::make_shared<fuse_constraints::AbsolutePose3DStampedConstraint>(
          "GLOBALMAPREGISTRATION", *p, *o, mean,
          params_.global_registration_covariance);
  transaction->addConstraint(prior);
  return transaction;
}

void LidarOdometry::onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph_msg) {
  updates_++;

  auto i = active_clouds_.begin();
  while (i != active_clouds_.end()) {
    bool update_successful = i->UpdatePose(graph_msg);
    if (update_successful) {
      // update map
      if (update_local_map_on_graph_update_) {
        local_scan_registration_->GetMapMutable().UpdateScan(
            i->Stamp(), i->T_REFFRAME_LIDAR());
      }

      // send reloc request if it is the first time the scan pose is updated,
      // and if the time elapsed since the last reloc request is greater than
      // the min.
      if (i->Updates() == 1 ||
          i->Stamp() - last_reloc_request_time_ >= reloc_request_period_) {
        SendRelocRequest(*i);
      }

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
    PublishMarginalizedScanPose(*i);
    if (!params_.scan_output_directory.empty()) {
      i->SaveCloud(params_.scan_output_directory);
    }
    active_clouds_.erase(i++);
  }

  if (!output_graph_updates_) { return; }

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

void LidarOdometry::process(const sensor_msgs::PointCloud2::ConstPtr& msg) {
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

void LidarOdometry::SendRelocRequest(const ScanPose& scan_pose) {
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
  pcl::transformPointCloud(scan_pose.Cloud(), cloud_in_baselink_frame,
                           T_BASELINK_LIDAR);
  beam_matching::LoamPointCloud loam_cloud_in_baselink_frame =
      scan_pose.LoamCloud();
  loam_cloud_in_baselink_frame.TransformPointCloud(T_BASELINK_LIDAR);

  // convert pose to vector
  const Eigen::Matrix4d& T = scan_pose.T_REFFRAME_BASELINK();
  std::vector<double> pose{T(0, 0), T(0, 1), T(0, 2), T(0, 3),
                           T(1, 0), T(1, 1), T(1, 2), T(1, 3),
                           T(2, 0), T(2, 1), T(2, 2), T(2, 3)};

  // create message and publish
  RelocRequestMsg msg;
  msg.stamp = scan_pose.Stamp();
  msg.T_WORLD_BASELINK = pose;
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

  last_reloc_request_time_ = scan_pose.Stamp();
}

void LidarOdometry::PublishMarginalizedScanPose(const ScanPose& scan_pose) {
  if (!params_.output_loam_points && params_.output_lidar_points) { return; }

  // output to global mapper
  SlamChunkMsg slam_chunk_msg;
  slam_chunk_msg.stamp = scan_pose.Stamp();

  std::vector<double> pose;
  const Eigen::Matrix4d& T = scan_pose.T_REFFRAME_BASELINK();
  for (uint8_t i = 0; i < 3; i++) {
    for (uint8_t j = 0; j < 4; j++) { pose.push_back(T(i, j)); }
  }

  slam_chunk_msg.T_WORLD_BASELINK = pose;
  slam_chunk_msg.lidar_measurement.frame_id = extrinsics_.GetLidarFrameId();

  if (params_.output_lidar_points) {
    // add regular points
    const PointCloud& cloud = scan_pose.Cloud();
    for (const auto& p : cloud) {
      geometry_msgs::Vector3 point;
      point.x = p.x;
      point.y = p.y;
      point.z = p.z;
      slam_chunk_msg.lidar_measurement.lidar_points.push_back(point);
    }
  }

  // if loam cloud not to be outputted, or empty, publish current msg
  const beam_matching::LoamPointCloud& loam_cloud = scan_pose.LoamCloud();
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

void LidarOdometry::PublishScanRegistrationResults(
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
          BEAM_WARN("UUID not found in registration map, not publishing scan "
                    "registration result.");
          continue;
        }
        Eigen::Matrix4d T_WORLD_REF;
        map.GetScanPose(ref_stamp, T_WORLD_REF);
        Ts_WORLD_LIDARREFLM.push_back(T_WORLD_REF * T_REF_SCAN);
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
  sensor_msgs::PointCloud2 msg_init = beam::PCLToROS(
      scan_in_world_init_frame, scan_pose.Stamp(),
      extrinsics_.GetWorldFrameId(), published_registration_results_);
  registration_publisher_init_.publish(msg_init);

  if (publish_lm_results) {
    PointCloud scan_in_world_lm_frame;

    pcl::transformPointCloud(scan_in_lidar_frame, scan_in_world_lm_frame,
                             T_WORLD_LIDARREFLM);
    sensor_msgs::PointCloud2 msg_align_lm = beam::PCLToROS(
        scan_in_world_lm_frame, scan_pose.Stamp(),
        extrinsics_.GetWorldFrameId(), published_registration_results_);

    registration_publisher_aligned_lm_.publish(msg_align_lm);
  }
  if (publish_gm_results) {
    PointCloud scan_in_world_gm_frame;
    pcl::transformPointCloud(scan_in_lidar_frame, scan_in_world_gm_frame,
                             T_WORLD_LIDARREFGM);
    sensor_msgs::PointCloud2 msg_align_gm = beam::PCLToROS(
        scan_in_world_gm_frame, scan_pose.Stamp(),
        extrinsics_.GetWorldFrameId(), published_registration_results_);
    registration_publisher_aligned_gm_.publish(msg_align_gm);
  }

  published_registration_results_++;
}

} // namespace bs_models
