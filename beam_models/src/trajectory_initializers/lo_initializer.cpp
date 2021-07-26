#include <beam_models/trajectory_initializers/lo_initializer.h>

#include <pluginlib/class_list_macros.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>

#include <beam_optimization/CeresParams.h>
#include <beam_utils/math.h>
#include <beam_utils/pointclouds.h>

#include <beam_models/InitializedPathMsg.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(beam_models::frame_to_frame::LoInitializer,
                       fuse_core::SensorModel)

namespace beam_models
{
  namespace frame_to_frame
  {

    using namespace beam_matching;

    LoInitializer::LoInitializer() : fuse_core::AsyncSensorModel(1) {}

    void LoInitializer::onInit()
    {
      // Read settings from the parameter sever
      params_.loadFromROS(private_node_handle_);

      // subscribe to lidar topic
      lidar_subscriber_ = private_node_handle_.subscribe(
          params_.lidar_topic, 100, &LoInitializer::processLidar, this);

      // init publisher
      results_publisher_ = private_node_handle_.advertise<InitializedPathMsg>(
          params_.output_topic, 1000);

      // initial scan registration
      std::shared_ptr<LoamParams> matcher_params =
          std::make_shared<LoamParams>(params_.matcher_params_path);

      // override iteration since we need this to be fast and scans are very close
      // to each other so iteration isn't necessary
      matcher_params->iterate_correspondences = false;

      // override ceres config
      beam_optimization::CeresParams ceres_params(params_.ceres_config_path);
      matcher_params->optimizer_params = ceres_params;

      matcher_ = std::make_unique<LoamMatcher>(*matcher_params);
      std::unique_ptr<beam_matching::Matcher<beam_matching::LoamPointCloudPtr>>
          matcher = std::make_unique<LoamMatcher>(*matcher_params);
      ScanToMapLoamRegistration::Params params;
      params.outlier_threshold_t = params_.outlier_threshold_t_m;
      params.outlier_threshold_r = params_.outlier_threshold_r_deg;
      params.map_size = params_.scan_registration_map_size;
      params.store_full_cloud = false;
      scan_registration_ =
          std::make_unique<ScanToMapLoamRegistration>(std::move(matcher), params);
      feature_extractor_ = std::make_shared<LoamFeatureExtractor>(matcher_params);

      // set covariance if not set to zero in config
      if (std::accumulate(params_.matcher_noise_diagonal.begin(),
                          params_.matcher_noise_diagonal.end(), 0.0) > 0)
      {
        Eigen::Matrix<double, 6, 6> covariance;
        covariance.setIdentity();
        for (int i = 0; i < 6; i++)
        {
          covariance(i, i) = params_.matcher_noise_diagonal[i];
        }
        scan_registration_->SetFixedCovariance(covariance);
      }
      else
      {
        scan_registration_->SetFixedCovariance(params_.matcher_noise);
      }

      // if outputting scans, clear folder
      if (!params_.scan_output_directory.empty())
      {
        if (boost::filesystem::is_directory(params_.scan_output_directory))
        {
          boost::filesystem::remove_all(params_.scan_output_directory);
        }
        boost::filesystem::create_directory(params_.scan_output_directory);
      }
    }

    void LoInitializer::processLidar(
        const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
      if (initialization_complete_)
      {
        return;
      }

      ROS_DEBUG("Received incoming scan");

      Eigen::Matrix4d T_WORLD_IMULAST;
      if (keyframes_.empty())
      {
        T_WORLD_IMULAST = Eigen::Matrix4d::Identity();
      }
      else
      {
        T_WORLD_IMULAST = keyframes_.back().T_REFFRAME_CLOUD();
      }

      PointCloudPtr cloud_current = beam::ROSToPCL(*msg);

      // init first message
      if (keyframe_start_time_.toSec() == 0)
      {
        ROS_DEBUG("Set first scan and imu start time.");
        keyframe_start_time_ = msg->header.stamp;
        if (!AddPointcloudToKeyframe(*cloud_current, msg->header.stamp))
        {
          keyframe_start_time_ = ros::Time(0);
        }
        prev_stamp_ = msg->header.stamp;
        return;
      }

      ros::Duration current_scan_period = msg->header.stamp - prev_stamp_;

      if (msg->header.stamp - keyframe_start_time_ + current_scan_period >
          params_.aggregation_time)
      {
        ProcessCurrentKeyframe();
        keyframe_cloud_.clear();
        keyframe_start_time_ = msg->header.stamp;
        keyframe_scan_counter_ = 0;
        T_WORLD_KEYFRAME_ = T_WORLD_IMULAST;
      }

      AddPointcloudToKeyframe(*cloud_current, msg->header.stamp);
      prev_stamp_ = msg->header.stamp;
    }

    void LoInitializer::onStop() {}

    void LoInitializer::ProcessCurrentKeyframe()
    {
      beam::HighResolutionTimer timer;
      if (keyframe_cloud_.size() == 0)
      {
        return;
      }

      ROS_DEBUG("Processing keyframe containing %d scans and %d points.",
                keyframe_scan_counter_, keyframe_cloud_.size());

      // create scan pose
      beam_common::ScanPose current_scan_pose(keyframe_start_time_,
                                              T_WORLD_KEYFRAME_, keyframe_cloud_,
                                              feature_extractor_);

      scan_registration_->RegisterNewScan(current_scan_pose);
      Eigen::Matrix4d T_WORLD_SCAN;
      bool scan_in_map = scan_registration_->GetMap().GetScanPose(
          current_scan_pose.Stamp(), T_WORLD_SCAN);
      if (scan_in_map)
      {
        current_scan_pose.Update(T_WORLD_SCAN);
        keyframes_.push_back(current_scan_pose);
      }
      else
      {
        return;
      }

      ROS_DEBUG("Total time to process keyframe: %.5f", timer.elapsed());

      // check if time window is full, if not then keep adding to the queue
      if (keyframes_.back().Stamp() - keyframes_.front().Stamp() <
          params_.trajectory_time_window)
      {
        ROS_DEBUG("Time windows not full, continuing to add keyframes.");
        return;
      }
      ROS_DEBUG("Time window is full, checking trajectory length.");

      // check that trajectory is long enough
      double trajectory_length = beam_common::CalculateTrajectoryLength(keyframes_);
      ROS_DEBUG("Trajectory length of %.3f m was calculated, with %d keyframes.",
                trajectory_length, keyframes_.size());

      if (trajectory_length > params_.min_trajectory_distance)
      {
        // if so, then optimize
        ROS_INFO("LO trajectory is long enough.");
        SetTrajectoryStart();
        PublishResults();
        OutputResults();
        initialization_complete_ = true;

        // clear lidar map so we can generate a new one during slam (it's a singleton)
        scan_registration_->GetMapMutable().Clear();

        ROS_INFO("Lo initialization complete");
        stop();
      }
      else
      {
        keyframes_.pop_front();
      }
    }

    bool LoInitializer::AddPointcloudToKeyframe(const PointCloud &cloud,
                                                const ros::Time &time)
    {
      Eigen::Matrix4d T_IMU_LIDAR;
      if (!extrinsics_.GetT_IMU_LIDAR(T_IMU_LIDAR, time))
      {
        ROS_WARN("Unable to get imu to lidar transform with time $.5f",
                 time.toSec());
        return false;
      }

      PointCloud cloud_converted;
      pcl::transformPointCloud(cloud, cloud_converted, T_IMU_LIDAR);
      keyframe_cloud_ += cloud_converted;
      keyframe_scan_counter_++;
      return true;
    }

    void LoInitializer::SetTrajectoryStart()
    {
      auto iter = keyframes_.begin();
      const Eigen::Matrix4d &T_WORLDOLD_KEYFRAME0 = iter->T_REFFRAME_CLOUD();
      Eigen::Matrix4d T_KEYFRAME0_WORLDOLD = beam::InvertTransform(T_WORLDOLD_KEYFRAME0);
      iter->Update(Eigen::Matrix4d::Identity());
      iter++;
      while (iter != keyframes_.end())
      {
        const Eigen::Matrix4d &T_WORLDOLD_KEYFRAMEX = iter->T_REFFRAME_CLOUD();
        Eigen::Matrix4d T_KEYFRAME0_KEYFRAMEX = T_KEYFRAME0_WORLDOLD * T_WORLDOLD_KEYFRAMEX;
        iter->Update(T_KEYFRAME0_KEYFRAMEX);
        iter++;
      }
    }

    void LoInitializer::OutputResults()
    {
      if (params_.scan_output_directory.empty())
      {
        return;
      }

      if (!boost::filesystem::exists(params_.scan_output_directory))
      {
        ROS_ERROR("Output directory does not exist. Not outputting LO Initializer results.");
        return;
      }

      ROS_DEBUG("Saving results to %s", params_.scan_output_directory.c_str());

      // create directories
      std::string save_path_init =
          params_.scan_output_directory + "/initial_poses/";
      boost::filesystem::create_directory(save_path_init);
      std::string save_path_final = params_.scan_output_directory + "/final_poses/";
      boost::filesystem::create_directory(save_path_final);

      // create frame for storing clouds
      PointCloudCol frame = beam::CreateFrameCol();

      // iterate through all keyframes, update based on graph and save initial and
      // final values
      for (auto iter = keyframes_.begin(); iter != keyframes_.end(); iter++)
      {
        const Eigen::Matrix4d &T_WORLD_SCAN_FIN = iter->T_REFFRAME_CLOUD();
        const Eigen::Matrix4d &T_WORLD_SCAN_INIT = iter->T_REFFRAME_CLOUD_INIT();
        PointCloud cloud_world_final;
        PointCloud cloud_world_init;
        pcl::transformPointCloud(iter->Cloud(), cloud_world_final,
                                 T_WORLD_SCAN_FIN);
        pcl::transformPointCloud(iter->Cloud(), cloud_world_init,
                                 T_WORLD_SCAN_INIT);

        PointCloudCol cloud_world_final_col =
            beam::ColorPointCloud(cloud_world_final, 0, 255, 0);
        PointCloudCol cloud_world_init_col =
            beam::ColorPointCloud(cloud_world_init, 255, 0, 0);

        beam::MergeFrameToCloud(cloud_world_final_col, frame, T_WORLD_SCAN_FIN);
        beam::MergeFrameToCloud(cloud_world_init_col, frame, T_WORLD_SCAN_INIT);

        std::string filename = std::to_string(iter->Stamp().toSec()) + ".pcd";
        pcl::io::savePCDFileASCII(save_path_final + filename,
                                  cloud_world_final_col);
        pcl::io::savePCDFileASCII(save_path_init + filename, cloud_world_init_col);
      }
    }

    void LoInitializer::PublishResults()
    {
      InitializedPathMsg msg;

      // get pose variables and add them to the msg
      int counter = 0;
      for (auto iter = keyframes_.begin(); iter != keyframes_.end(); iter++)
      {
        std_msgs::Header header;
        header.frame_id = extrinsics_.GetImuFrameId();
        header.seq = counter;
        header.stamp = iter->Stamp();

        const Eigen::Matrix4d &T = iter->T_REFFRAME_CLOUD();

        geometry_msgs::Point position;
        position.x = T(0, 3);
        position.y = T(1, 3);
        position.z = T(2, 3);

        Eigen::Matrix3d R = T.block(0, 0, 3, 3);
        Eigen::Quaterniond q(R);

        geometry_msgs::Quaternion orientation;
        orientation.x = q.x();
        orientation.y = q.y();
        orientation.z = q.z();
        orientation.w = q.w();

        geometry_msgs::PoseStamped pose;
        pose.header = header;
        pose.pose.position = position;
        pose.pose.orientation = orientation;
        msg.poses.push_back(pose);
        counter++;
      }

      results_publisher_.publish(msg);
    }

  } // namespace frame_to_frame
} // namespace beam_models
