#include <bs_models/lidar_odometry.h>

#include <filesystem>

#include <fuse_core/transaction.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Time.h>

#include <beam_utils/filesystem.h>

#include <bs_common/bs_msgs.h>
#include <bs_common/conversions.h>
#include <bs_models/frame_initializers/frame_initializer.h>
#include <bs_models/graph_visualization/helpers.h>
#include <bs_models/scan_registration/multi_scan_registration.h>
#include <bs_models/scan_registration/scan_to_map_registration.h>
#include <bs_variables/orientation_3d.h>
#include <bs_variables/position_3d.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(bs_models::LidarOdometry, fuse_core::SensorModel)

namespace bs_models {

using namespace scan_registration;
using namespace beam_matching;

LidarOdometry::LidarOdometry()
    : fuse_core::AsyncSensorModel(3),
      device_id_(fuse_core::uuid::NIL),
      throttled_callback_(
          std::bind(&LidarOdometry::process, this, std::placeholders::_1)) {}

void LidarOdometry::onInit() {
  params_.loadFromROS(private_node_handle_);

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
    if (!std::filesystem::is_directory(params_.scan_output_directory)) {
      std::filesystem::create_directory(params_.scan_output_directory);
    }

    if (params_.save_graph_updates) {
      graph_updates_path_ =
          beam::CombinePaths(params_.scan_output_directory, "graph_updates");
      if (std::filesystem::is_directory(graph_updates_path_)) {
        std::filesystem::remove_all(graph_updates_path_);
      }
      std::filesystem::create_directory(graph_updates_path_);
    }

    if (params_.save_scan_registration_results) {
      registration_results_path_ = beam::CombinePaths(
          params_.scan_output_directory, "registration_results");
      if (std::filesystem::is_directory(registration_results_path_)) {
        std::filesystem::remove_all(registration_results_path_);
      }
      std::filesystem::create_directory(registration_results_path_);
    }

    if (params_.save_marginalized_scans) {
      marginalized_scans_path_ = beam::CombinePaths(
          params_.scan_output_directory, "marginalized_scans");
      if (std::filesystem::is_directory(marginalized_scans_path_)) {
        std::filesystem::remove_all(marginalized_scans_path_);
      }
      std::filesystem::create_directory(marginalized_scans_path_);
    }
  }
}

void LidarOdometry::onStart() {
  subscriber_ = private_node_handle_.subscribe<sensor_msgs::PointCloud2>(
      ros::names::resolve(params_.input_topic), 10,
      &ThrottledCallback::callback, &throttled_callback_,
      ros::TransportHints().tcpNoDelay(false));

  if (params_.output_loam_points || params_.output_lidar_points) {
    results_publisher_ =
        private_node_handle_.advertise<bs_common::SlamChunkMsg>(
            "/local_mapper/slam_results", 100);
  }

  // odometry publishers
  odom_publisher_ =
      private_node_handle_.advertise<nav_msgs::Odometry>("odometry", 100);
  odom_publisher_marginalized_ =
      private_node_handle_.advertise<nav_msgs::Odometry>("marginalized", 100);
  imu_constraint_trigger_publisher_ =
      private_node_handle_.advertise<std_msgs::Time>(
          "/local_mapper/inertial_odometry/trigger", 10);

  std::string baselink_frame = extrinsics_.GetBaselinkFrameId();
  std::string lidar_frame = extrinsics_.GetLidarFrameId();
  bs_variables::Position3D p_tmp;
  extrinsics_position_uuid_ =
      fuse_core::uuid::generate(p_tmp.type(), baselink_frame + lidar_frame);
  bs_variables::Orientation3D o_tmp;
  extrinsics_orientation_uuid_ =
      fuse_core::uuid::generate(o_tmp.type(), baselink_frame + lidar_frame);
}

void LidarOdometry::onStop() {
  // if output set, save scans before stopping
  ROS_INFO("LidarOdometry stopped, processing remaining scans in window.");
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

void LidarOdometry::SetupRegistration() {
  // setup registration
  beam_matching::MatcherType matcher_type;
  if (!params_.matcher_config.empty()) {
    const auto& reg_filepath = params_.registration_config;
    const auto& matcher_filepath = params_.matcher_config;
    scan_registration_ = ScanRegistrationBase::Create(
        reg_filepath, matcher_filepath, registration_results_path_, false);

    // setup feature extractor if needed
    matcher_type = beam_matching::GetTypeFromConfig(matcher_filepath);
    if (matcher_type == beam_matching::MatcherType::LOAM) {
      std::string ceres_config = bs_common::GetAbsoluteConfigPathFromJson(
          matcher_filepath, "ceres_config");
      std::shared_ptr<LoamParams> matcher_params =
          std::make_shared<LoamParams>(matcher_filepath, ceres_config);
      feature_extractor_ =
          std::make_shared<LoamFeatureExtractor>(matcher_params);
    }
  }

  // set registration map to publish
  RegistrationMap& map = RegistrationMap::GetInstance();
  if (params_.publish_registration_map) {
    map.SetParams(map.MapSize(), true);
    ROS_INFO("Publishing initial lidar_odometry registration map");
    map.Publish();
  }

  // Get last scan pose to initialize with if registration map isn't empty
  if (!map.Empty()) {
    // get extrinsics
    Eigen::Matrix4d T_Lidar_Baselink;
    if (!extrinsics_.GetT_LIDAR_BASELINK(T_Lidar_Baselink)) {
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

    T_World_BaselinkLast_ = T_MAP_SCAN * T_Lidar_Baselink;
    last_map_update_time_ = ros::Time::now();
  }

  scan_registration_->SetInformationWeight(params_.lidar_information_weight);
}

void LidarOdometry::onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph_msg) {
  if (updates_ == 0) {
    ROS_INFO("received first graph update, initializing registration and "
             "starting lidar odometry");
    SetupRegistration();

    const auto timestamps = bs_common::CurrentTimestamps(*graph_msg);
    for (const auto& t : timestamps) {
      auto maybe_p = bs_common::GetPosition(*graph_msg, t);
      auto maybe_o = bs_common::GetOrientation(*graph_msg, t);
      if (maybe_p && maybe_o) {
        Eigen::Matrix4d T_WORLD_BASELINK =
            bs_common::FusePoseToEigenTransform(*maybe_p, *maybe_o);
        nav_msgs::Odometry odom_msg = bs_common::TransformToOdometryMessage(
            t, odom_publisher_counter_, extrinsics_.GetWorldFrameId(),
            extrinsics_.GetBaselinkFrameId(), T_WORLD_BASELINK);
        odom_publisher_.publish(odom_msg);
        odom_publisher_counter_++;
      }
    }
  }

  updates_++;
  PublishExtrinsics(graph_msg);

  // update map
  if (update_registration_map_all_scans_) {
    scan_registration_->GetMapMutable().UpdateScanPosesFromGraphMsg(graph_msg);
  } else if (update_registration_map_in_batch_) {
    ros::Time now = ros::Time::now();
    if (now >= (last_map_update_time_ + registration_map_batch_update_dur_)) {
      last_map_update_time_ = now;
      Eigen::Matrix4d T_WorldCorrected_World =
          scan_registration_->GetMapMutable().CorrectMapDriftFromGraphMsg(
              graph_msg);
      T_World_BaselinkLast_ = T_WorldCorrected_World * T_World_BaselinkLast_;
    }
  }

  // the remainder just publishes and/or saves results.
  auto i = active_clouds_.begin();
  while (i != active_clouds_.end()) {
    std::shared_ptr<ScanPose>& scan_pose = *i;
    bool update_successful = scan_pose->UpdatePose(graph_msg);
    if (update_successful) {
      ++i;
      continue;
    }

    // if scan has never been updated, then it is probably just not yet in the
    // window so do nothing
    if (scan_pose->Updates() == 0) {
      ++i;
      continue;
    }

    // Otherwise, it has probably been marginalized out, so output and remove
    // from active list
    PublishMarginalizedScanPose(*i);
    if (params_.save_marginalized_scans) {
      scan_pose->SaveCloud(marginalized_scans_path_);
    }
    active_clouds_.erase(i++);
  }

  if (params_.save_graph_updates) {
    std::string update_time =
        beam::ConvertTimeToDate(std::chrono::system_clock::now());
    std::string curent_path =
        beam::CombinePaths(graph_updates_path_,
                           "U" + std::to_string(updates_) + "_" + update_time);
    std::filesystem::create_directory(curent_path);
    for (auto iter = active_clouds_.begin(); iter != active_clouds_.end();
         iter++) {
      (*iter)->SaveCloud(curent_path);
    }
  }
}

void LidarOdometry::process(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  if (updates_ == 0) {
    ROS_INFO_THROTTLE(
        1, "lidar odometry not yet initialized, waiting on first graph "
           "update before beginning");
    return;
  }

  // buffer the message
  scan_buffer_.push_back(msg);

  while (!scan_buffer_.empty() && scan_buffer_.size() > max_scan_buffer_size_) {
    ROS_WARN("Lidar buffer size exceeded max of %d, removing last scan",
             max_scan_buffer_size_);
    scan_buffer_.pop_front();
  }

  while (!scan_buffer_.empty()) {
    const auto current_msg = scan_buffer_.front();

    // ensure monotonically increasing data
    if (current_msg->header.stamp <= last_scan_pose_time_) {
      ROS_WARN(
          "detected non-monotonically increasing lidar stamp, skipping scan");
      scan_buffer_.pop_front();
      break;
    }

    Eigen::Matrix4d T_World_BaselinkInit;
    bool init_successful{true};
    std::string error_msg;

    if (frame_initializer_ == nullptr) {
      T_World_BaselinkInit = T_World_BaselinkLast_;
    } else if (use_frame_init_relative_) {
      Eigen::Matrix4d T_BaselinkLast_BaselinkCurrent;
      init_successful = frame_initializer_->GetRelativePose(
          T_BaselinkLast_BaselinkCurrent, last_scan_pose_time_,
          current_msg->header.stamp, error_msg);
      T_World_BaselinkInit =
          T_World_BaselinkLast_ * T_BaselinkLast_BaselinkCurrent;
    } else {
      init_successful = frame_initializer_->GetPose(
          T_World_BaselinkInit, current_msg->header.stamp,
          extrinsics_.GetBaselinkFrameId(), error_msg);
    }

    if (!init_successful) {
      ROS_DEBUG("Could not initialize frame, buffering scan. Reason: %s",
                error_msg.c_str());
      break;
    }
    Eigen::Matrix4d T_Baselink_Lidar;
    if (!extrinsics_.GetT_BASELINK_LIDAR(T_Baselink_Lidar, ros::Time::now())) {
      ROS_ERROR("Cannot get transform from lidar to baselink for stamp: %.8f. "
                "Buffering scan.",
                current_msg->header.stamp.toSec());
      break;
    }

    std::shared_ptr<ScanPose> current_scan_pose;
    if (params_.lidar_type == LidarType::VELODYNE) {
      pcl::PointCloud<PointXYZIRT> cloud_current_unfiltered;
      beam::ROSToPCL(cloud_current_unfiltered, *msg);
      pcl::PointCloud<PointXYZIRT> cloud_filtered =
          beam_filtering::FilterPointCloud<PointXYZIRT>(
              cloud_current_unfiltered, input_filter_params_);
      current_scan_pose = std::make_shared<ScanPose>(
          cloud_filtered, current_msg->header.stamp, T_World_BaselinkInit,
          T_Baselink_Lidar, feature_extractor_);
    } else if (params_.lidar_type == LidarType::OUSTER) {
      pcl::PointCloud<PointXYZITRRNR> cloud_current_unfiltered;
      beam::ROSToPCL(cloud_current_unfiltered, *msg);
      pcl::PointCloud<PointXYZITRRNR> cloud_filtered =
          beam_filtering::FilterPointCloud(cloud_current_unfiltered,
                                           input_filter_params_);
      current_scan_pose = std::make_shared<ScanPose>(
          cloud_filtered, current_msg->header.stamp, T_World_BaselinkInit,
          T_Baselink_Lidar, feature_extractor_);
    } else {
      ROS_ERROR(
          "Invalid lidar type param. Lidar type may not be implemented yet.");
      throw std::runtime_error(
          "Invalid lidar type param. Lidar type may not be implemented yet.");
    }

    Eigen::Matrix4d T_World_BaselinkCurrent;
    fuse_core::Transaction::SharedPtr transaction;
    transaction = scan_registration_->RegisterNewScan(*current_scan_pose)
                      .GetTransaction();

    Eigen::Matrix4d T_WORLD_LIDAR;
    scan_registration_->GetMap().GetScanPose(current_scan_pose->Stamp(),
                                             T_WORLD_LIDAR);
    T_World_BaselinkCurrent = T_WORLD_LIDAR * T_Baselink_Lidar;

    if (transaction == nullptr) {
      ROS_WARN("No transaction generated, skipping scan.");
      scan_buffer_.pop_front();
      break;
    }
    sendTransaction(transaction);

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
      sendTransaction(prior_transaction);
    }

    // send IO trigger
    if (params_.trigger_inertial_odom_constraints) {
      std_msgs::Time time_msg;
      time_msg.data = current_scan_pose->Stamp();
      imu_constraint_trigger_publisher_.publish(time_msg);
      imu_constraint_trigger_counter_++;
    }

    active_clouds_.push_back(current_scan_pose);

    // publish odom
    nav_msgs::Odometry odom_msg;
    bs_common::EigenTransformToOdometryMsg(
        T_World_BaselinkCurrent, current_scan_pose->Stamp(),
        odom_publisher_counter_, extrinsics_.GetWorldFrameId(),
        extrinsics_.GetBaselinkFrameId(), odom_msg);
    odom_publisher_.publish(odom_msg);
    odom_publisher_counter_++;
    PublishTfTransform(T_World_BaselinkCurrent, "lidar_world",
                       extrinsics_.GetBaselinkFrameId(),
                       current_scan_pose->Stamp());

    // set current measurements to last
    T_World_BaselinkLast_ = T_World_BaselinkCurrent;
    last_scan_pose_time_ = current_scan_pose->Stamp();

    scan_buffer_.pop_front();
  }
}

void LidarOdometry::PublishMarginalizedScanPose(
    const std::shared_ptr<ScanPose>& scan_pose) {
  nav_msgs::Odometry odom_msg;
  bs_common::EigenTransformToOdometryMsg(
      scan_pose->T_REFFRAME_BASELINK(), scan_pose->Stamp(),
      odom_publisher_marginalized_counter_, "lidar_world_marginalized",
      extrinsics_.GetBaselinkFrameId(), odom_msg);
  odom_publisher_marginalized_.publish(odom_msg);
  odom_publisher_marginalized_counter_++;

  if (!params_.output_loam_points && !params_.output_lidar_points) { return; }

  // output to global mapper
  bs_common::SlamChunkMsg slam_chunk_msg;
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

void LidarOdometry::PublishTfTransform(const Eigen::Matrix4d& T_Child_Parent,
                                       const std::string& child_frame,
                                       const std::string& parent_frame,
                                       const ros::Time& time) {
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(T_Child_Parent(0, 3), T_Child_Parent(1, 3),
                                  T_Child_Parent(2, 3)));

  Eigen::Matrix3d R = T_Child_Parent.block(0, 0, 3, 3);
  Eigen::Quaterniond q(R);
  tf::Quaternion q_tf(q.x(), q.y(), q.z(), q.w());
  transform.setRotation(q_tf);
  tf_broadcaster_.sendTransform(
      tf::StampedTransform(transform, time, parent_frame, child_frame));
}

void LidarOdometry::PublishExtrinsics(
    fuse_core::Graph::ConstSharedPtr graph_msg) {
  if (!publish_extrinsics_) { return; }
  const auto var_range = graph_msg->getVariables();
  if (!graph_msg->variableExists(extrinsics_position_uuid_) ||
      !graph_msg->variableExists(extrinsics_orientation_uuid_)) {
    ROS_WARN_THROTTLE(5, "No extrinsics variables found for Lidar.");
    return;
  }

  auto p = dynamic_cast<const bs_variables::Position3D&>(
      graph_msg->getVariable(extrinsics_position_uuid_));
  auto o = dynamic_cast<const bs_variables::Orientation3D&>(
      graph_msg->getVariable(extrinsics_orientation_uuid_));

  tf::Transform transform;
  transform.setOrigin(tf::Vector3(p.x(), p.y(), p.z()));
  tf::Quaternion q(o.x(), o.y(), o.z(), o.w());
  transform.setRotation(q);
  tf_broadcaster_.sendTransform(tf::StampedTransform(
      transform, ros::Time::now(), extrinsics_.GetLidarFrameId(),
      extrinsics_.GetBaselinkFrameId()));
}

} // namespace bs_models
