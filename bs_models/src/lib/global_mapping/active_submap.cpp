#include <bs_models/global_mapping/active_submap.h>

#include <pcl/common/transforms.h>

#include <beam_cv/descriptors/Descriptor.h>
#include <beam_utils/math.h>
#include <beam_utils/pointclouds.h>

namespace bs_models {

ActiveSubmap::ActiveSubmap() {
  ros::NodeHandle n;

  // setup subsriber
  submap_subscriber_ = n.subscribe("/global_mapper/active_submap", 1,
                                   &ActiveSubmap::ActiveSubmapCallback, this);

  // setup publishers
  visual_map_publisher_ =
      n.advertise<sensor_msgs::PointCloud2>("/active_submap/visual_map", 10);
  lidar_map_publisher_ =
      n.advertise<sensor_msgs::PointCloud2>("/active_submap/lidar_map", 10);
  loam_map_publisher_ =
      n.advertise<sensor_msgs::PointCloud2>("/active_submap/loam_map", 10);

  // instantitate pointers
  visual_map_points_ = std::make_shared<PointCloud>();
  lidar_map_points_ = std::make_shared<PointCloud>();
  loam_cloud_ = std::make_shared<beam_matching::LoamPointCloud>();
}

ActiveSubmap& ActiveSubmap::GetInstance() {
  static ActiveSubmap instance;
  return instance;
}

void ActiveSubmap::ActiveSubmapCallback(
    const bs_common::SubmapMsg::ConstPtr& msg) {
  updates_counter_++;
  update_time_ = ros::Time::now();

  descriptors_.clear();
  visual_map_points_->clear();
  lidar_map_points_->clear();

  // get descriptor type
  beam_cv::DescriptorType d_type =
      beam_cv::DescriptorTypeIntMap[msg->descriptor_type];

  // add all descriptors to list
  for (auto& d : msg->visual_map_descriptors) {
    cv::Mat desc = beam_cv::Descriptor::CreateDescriptor(d.data, d_type);
    descriptors_.push_back(desc);
  }

  // add all 3d locations of landmarks to cloud
  for (auto& p : msg->visual_map_points) {
    pcl::PointXYZ point(p.x, p.y, p.z);
    visual_map_points_->push_back(point);
  }

  // if lidar map not empty, check frame id
  if (!msg->lidar_map.lidar_points.empty() ||
      !msg->lidar_map.lidar_edges_strong.empty() ||
      !msg->lidar_map.lidar_surfaces_strong.empty()) {
    if (msg->lidar_map.frame_id != extrinsics_online_.GetWorldFrameId()) {
      BEAM_WARN(
          "Lidar measurement frame id in submap msg not consistent with world "
          "frame in extrinsics.");
    }
  }

  // add all lidar points to point cloud
  for (geometry_msgs::Vector3 point_vec : msg->lidar_map.lidar_points) {
    pcl::PointXYZ point(point_vec.x, point_vec.y, point_vec.z);
    lidar_map_points_->push_back(point);
  }

  // add loam pointcloud
  PointCloudIRT edges_strong =
      beam::ROSVectorToPCLIRT(msg->lidar_map.lidar_edges_strong);
  PointCloudIRT edges_weak =
      beam::ROSVectorToPCLIRT(msg->lidar_map.lidar_edges_weak);
  PointCloudIRT surfaces_strong =
      beam::ROSVectorToPCLIRT(msg->lidar_map.lidar_surfaces_strong);
  PointCloudIRT surfaces_weak =
      beam::ROSVectorToPCLIRT(msg->lidar_map.lidar_surfaces_weak);
  loam_cloud_ = std::make_shared<beam_matching::LoamPointCloud>(
      edges_strong, surfaces_strong, edges_weak, surfaces_weak);

  if (publish_updates_) { Publish(); }
}

void ActiveSubmap::SetPublishUpdates(bool publish_updates) {
  publish_updates_ = publish_updates;
}

std::vector<Eigen::Vector3d> ActiveSubmap::GetVisualMapVectorInCameraFrame(
    const Eigen::Matrix4d& T_WORLD_CAMERA) const {
  PointCloud cloud_in_cam_frame =
      GetVisualMapCloudInCameraFrame(T_WORLD_CAMERA);

  std::vector<Eigen::Vector3d> vector_in_cam_frame;
  for (auto it = visual_map_points_->begin(); it != visual_map_points_->end();
       it++) {
    vector_in_cam_frame.push_back(Eigen::Vector3d{it->x, it->y, it->z});
  }
  return vector_in_cam_frame;
}

PointCloud ActiveSubmap::GetVisualMapCloudInCameraFrame(
    const Eigen::Matrix4d& T_WORLD_CAMERA) const {
  if (T_WORLD_CAMERA.isIdentity()) { return *visual_map_points_; }

  PointCloud cloud_in_cam_frame;
  pcl::transformPointCloud(*visual_map_points_, cloud_in_cam_frame,
                           beam::InvertTransform(T_WORLD_CAMERA));
  return cloud_in_cam_frame;
}

const PointCloudPtr ActiveSubmap::GetVisualMapPoints() const {
  return visual_map_points_;
}

const std::vector<cv::Mat>& ActiveSubmap::GetDescriptors() const {
  return descriptors_;
}

const PointCloudPtr ActiveSubmap::GetLidarMap() const {
  return lidar_map_points_;
}

const beam_matching::LoamPointCloudPtr ActiveSubmap::GetLoamMapPtr() const {
  return loam_cloud_;
}

void ActiveSubmap::RemoveVisualMapPoint(size_t index) {
  visual_map_points_->erase(visual_map_points_->begin() + index);
  descriptors_.erase(descriptors_.begin() + index);
}

void ActiveSubmap::Publish() const {
  std::string frame_id = extrinsics_online_.GetWorldFrameId();

  if (!visual_map_points_->empty()) {
    sensor_msgs::PointCloud2 pc_msg = beam::PCLToROS<pcl::PointXYZ>(
        *visual_map_points_, update_time_, frame_id, updates_counter_);
    visual_map_publisher_.publish(pc_msg);
  }

  if (!lidar_map_points_->empty()) {
    sensor_msgs::PointCloud2 pc_msg = beam::PCLToROS<pcl::PointXYZ>(
        *lidar_map_points_, update_time_, frame_id, updates_counter_);
    lidar_map_publisher_.publish(pc_msg);
  }

  if (!loam_cloud_->Empty()) {
    beam_matching::LoamPointCloudCombined loam_combined = loam_cloud_->GetCombinedCloud();
    sensor_msgs::PointCloud2 pc_msg = beam::PCLToROS<PointLoam>(
        loam_combined, update_time_, frame_id, updates_counter_);
    loam_map_publisher_.publish(pc_msg);
  }

}

} // namespace bs_models