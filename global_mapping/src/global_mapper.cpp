#include <global_mapping/global_mapper.h>
#include <global_mapping/SubmapMsg.h>
#include <global_mapping/SubmapRequestMsg.h>

#include <fuse_core/transaction.h>
#include <pluginlib/class_list_macros.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(global_mapping::GlobalMapper, fuse_core::SensorModel)

namespace global_mapping
{

  GlobalMapper::GlobalMapper()
      : fuse_core::AsyncSensorModel(1),
        device_id_(fuse_core::uuid::NIL),
        throttled_callback_(
            std::bind(&GlobalMapper::process, this, std::placeholders::_1)) {}

  void GlobalMapper::process(const SlamChunkMsg::ConstPtr &msg)
  {
    fuse_core::Transaction::SharedPtr new_transaction1 =
        global_map_->AddCameraMeasurement(msg->camera_measurement);
    if (new_transaction1 != nullptr)
    {
      sendTransaction(new_transaction1);
    }
    fuse_core::Transaction::SharedPtr new_transaction2 =
        global_map_->AddLidarMeasurement(msg->lidar_measurement);
    if (new_transaction2 != nullptr)
    {
      sendTransaction(new_transaction2);
    }
    fuse_core::Transaction::SharedPtr new_transaction3 =
        global_map_->AddTrajectoryMeasurement(msg->trajectory_measurement);
    if (new_transaction3 != nullptr)
    {
      sendTransaction(new_transaction3);
    }
  }

  void GlobalMapper::onInit()
  {
    params_.loadFromROS(private_node_handle_);

    submap_request_subscriber_ = private_node_handle_.subscribe("submap_request", 10, &onSubmapRequest);
    submap_publisher_ = private_node_handle_.advertise<SubmapMsg>("submap", 1);

    std::shared_ptr<beam_calibration::CameraModel> camera_model =
        beam_calibration::CameraModel::Create(params_.intrinsics_path);
    if (!params_.global_mapper_config.empty())
    {
      global_map_ = std::make_unique<GlobalMap>(camera_model,
                                                params_.global_mapper_config);
    }
    else
    {
      global_map_ = st

          ros::Subscriber submap_request_subscriber_;
      d::make_unique<GlobalMap>(camera_model);
    }
  }

  void GlobalMapper::onStop()
  {
    global_map_->SaveTrajectoryFile(params_.output_path,
                                    params_.save_local_mapper_trajectory);
    if (params_.save_trajectory_cloud)
    {
      global_map_->SaveTrajectoryClouds(params_.output_path,
                                        params_.save_local_mapper_trajectory);
    }
    if (params_.save_submaps)
    {
      global_map_->SaveLidarSubmaps(params_.output_path,
                                    params_.save_local_mapper_maps);
      global_map_->SaveKeypointSubmaps(params_.output_path,
                                       params_.save_local_mapper_maps);
    }
    if (params_.save_final_map)
    {
      global_map_->SaveFullLidarMap(params_.output_path,
                                    params_.save_local_mapper_maps);
      global_map_->SaveFullKeypointMap(params_.output_path,
                                       params_.save_local_mapper_maps);
    }
  }

  void GlobalMapper::onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph_msg)
  {
    global_map_->UpdateSubmapPoses(graph_msg);
  }

  void GlbobalMapper::onSubmapRequest(const SubmapRequestMsg::ConstPtr &request)
  {
    // request->scan: recent scan
    // request->image: recent image
    // request->pose_estimate: pose estimate
    // request->stamp: time stamp
    std::cout << "Submap request message with time stamp " << request->stamp << " received" << std::endl;

    // publish if reloc changes operating submap
    if (true)
    {
      std::cout << "Submap has changed.  Publishing to topic \"submap\"" << std::endl;
      global_mapping::SubmapMsg msg;
      msg.stamp = ros::Time::now();
      msg.submap = [];
      msg.keypoints = [];
      submap_publisher_.publish(msg);
    }
  }

} // namespace global_mapping
