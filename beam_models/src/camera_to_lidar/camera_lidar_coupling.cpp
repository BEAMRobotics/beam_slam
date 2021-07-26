#include <unordered_map>

#include <beam_models/camera_to_lidar/camera_lidar_coupling.h>
#include <beam_constraints/camera_to_lidar/camera_lidar_constraint.h>

#include <fuse_core/transaction.h>
#include <fuse_variables/point_3d_landmark.h>
#include <pluginlib/class_list_macros.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(beam_models::camera_to_lidar::CameraLidarCoupling,
                       fuse_core::SensorModel)

namespace beam_models {
namespace camera_to_lidar {

CameraLidarCoupling::CameraLidarCoupling()
    : fuse_core::AsyncSensorModel(1),
      device_id_(fuse_core::uuid::NIL),
      throttled_callback_(std::bind(&CameraLidarCoupling::process, this,
                                    std::placeholders::_1)) {}

void CameraLidarCoupling::onInit() {
  params_.loadFromROS(private_node_handle_);

  // if outputting correspondences, clear folder
  if (output_correspondences_) {
    BEAM_INFO("Clearing output directory: {}", output_path_);
    if (boost::filesystem::is_directory(output_path_)) {
      boost::filesystem::remove_all(output_path_);
    }
    boost::filesystem::create_directory(output_path_);
  }
}

void CameraLidarCoupling::onStart() {
  subscriber_ = node_handle_.subscribe(params_.input_topic, 1000,
                                       &ThrottledCallback::callback,
                                       &throttled_callback_);
};

void CameraLidarCoupling::onStop() { subscriber_.shutdown(); }

void CameraLidarCoupling::onGraphUpdate(
    fuse_core::Graph::ConstSharedPtr graph_msg) {
  // get transaction for constaints on all new keypoints in the queue
  fuse_core::Transaction::SharedPtr new_transaction =
      ProcessKeypointsQueue(graph_msg);
  if (new_transaction != nullptr) {
    ROS_DEBUG("Sending transaction of camera-lidar keypoints.");
    sendTransaction(new_transaction);
  }
}

void CameraLidarCoupling::process(const std_msgs::Int64::ConstPtr& msg) {
  keypoints_queue_.push(msg->data);
}

fuse_core::Transaction::SharedPtr CameraLidarCoupling::ProcessKeypointsQueue(
    fuse_core::Graph::ConstSharedPtr graph_msg) {
  fuse_variables::Point3DLandmark position_var_empty;
  std::vector<uint64_t> missing_landmarks;

  // get all landmarks in the graph message and save as unordered map
  std::unordered_map<fuse_core::UUID,
                     fuse_variables::Point3DLandmark::SharedPtr>
      existing_keypoints;
  for (auto& var : graph_msg->getVariables()) {
    fuse_variables::Point3DLandmark::SharedPtr landmark =
        fuse_variables::Point3DLandmark::make_shared();
    if (var.type() == position_var_empty.type()) {
      *landmark = dynamic_cast<const fuse_variables::Point3DLandmark&>(var);
      existing_keypoints.emplace(landmark->uuid(), landmark);
    }
  }

  fuse_core::Transaction::SharedPtr transaction =
      fuse_core::Transaction::make_shared();

  // get kd search tree for current lidar map
  pcl::KdTreeFLANN<pcl::PointXYZ> lidar_map_search_tree;
  const PointCloud& cloud = lidar_map_.GetPointCloudMap();
  lidar_map_search_tree.setInputCloud(std::make_shared<PointCloud>(cloud));

  while (keypoints_queue_.size() > 0) {
    uint64_t landmark_id = keypoints_queue_.front();
    keypoints_queue_.pop();
    fuse_core::UUID landmark_uuid = fuse_core::uuid::generate(
        position_var_empty.type(), std::to_string(landmark_id).c_str());

    auto iter = existing_keypoints.find(landmark_uuid);
    if (iter == existing_keypoints.end()) {
      missing_landmarks.push_back(landmark_id);
      continue;
    }

    fuse_variables::Point3DLandmark::SharedPtr landmark = iter->second;
    Eigen::Vector3d lidar_point;
    bool success =
        GetLidarCorrespondence(landmark, lidar_map_search_tree, cloud, lidar_point);

    if (!success) {
      missing_landmarks.push_back(landmark_id);
      continue;
    }

    beam_constraints::CameraLidarConstraint::SharedPtr camera_lidar_constraint =
        beam_constraints::CameraLidarConstraint::make_shared(source_, *landmark,
                                                             lidar_point);
    transaction->addConstraint(camera_lidar_constraint);
  }

  // go back and add all missing landmarks to the queue
  for (uint64_t id : missing_landmarks) {
    keypoints_queue_.push(id);
  }

  return transaction;
}

bool CameraLidarCoupling::GetLidarCorrespondence(
    const fuse_variables::Point3DLandmark::SharedPtr& landmark,
    const pcl::KdTreeFLANN<pcl::PointXYZ>& lidar_map_search_tree,
    const PointCloud& lidar_map, Eigen::Vector3d& lidar_point) {
  pcl::PointXYZ landmark_pcl;
  landmark_pcl.x = static_cast<float>(landmark->x());
  landmark_pcl.y = static_cast<float>(landmark->y());
  landmark_pcl.z = static_cast<float>(landmark->z());
  std::vector<int> indices;
  std::vector<float> distances;
  int num_returned =
      lidar_map_search_tree.nearestKSearch(landmark_pcl, 1, indices, distances);

  if (num_returned != 1) {
    return false;
  }

  if (distances[0] > params_.correspondence_distance_theshold) {
    return false;
  }

  const pcl::PointXYZ& corresponding_point = lidar_map.points.at(indices[0]);

  lidar_point[0] = corresponding_point.x;
  lidar_point[1] = corresponding_point.y;
  lidar_point[2] = corresponding_point.z;
}

}  // namespace camera_to_lidar
}  // namespace beam_models
