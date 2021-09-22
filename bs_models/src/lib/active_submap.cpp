#include <bs_models/active_submap.h>

#include <beam_cv/descriptors/Descriptor.h>

namespace bs_models {

ActiveSubmap::ActiveSubmap() {
  ros::NodeHandle n;
  submap_subscriber_ = n.subscribe("/active_submap", 1,
                                   &ActiveSubmap::ActiveSubmapCallback, this);
}

ActiveSubmap& ActiveSubmap::GetInstance() {
  static ActiveSubmap instance;
  return instance;
}

void ActiveSubmap::ActiveSubmapCallback(
    const bs_common::SubmapMsg::ConstPtr& msg) {
  descriptors_.clear();
  visual_map_points_.clear();
  point_cloud_.clear();

  // get descriptor type
  beam_cv::DescriptorType d_type =
      beam_cv::DescriptorTypeIntMap[msg->descriptor_type];

  // add all descriptors to list
  for (auto& d : msg->visual_map_descriptors) {
    cv::Mat desc = beam_cv::Descriptor::CreateDescriptor(d.data, d_type);
    descriptors_.push_back(desc);
  }

  // add all 3d locations of landmarks to list
  for (auto& p : msg->visual_map_points) {
    Eigen::Vector3d point{p.x, p.y, p.z};
    visual_map_points_.push_back(point);
  }

  // add all lidar points to point cloud
  if (msg->lidar_map.size() > 0) {
    for (geometry_msgs::Vector3 point_vec : msg->lidar_map) {
      pcl::PointXYZ point(point_vec.x, point_vec.y, point_vec.z);
      point_cloud_.push_back(point);
    }
  }

  // loam pointcloud
  PointCloud edges_strong;
  PointCloud edges_weak;
  PointCloud surfaces_strong;
  PointCloud surfaces_weak;
  for (geometry_msgs::Vector3 point_vec : msg->lidar_edges_strong) {
    pcl::PointXYZ point(point_vec.x, point_vec.y, point_vec.z);
    edges_strong.push_back(point);
  }
  for (geometry_msgs::Vector3 point_vec : msg->lidar_edges_weak) {
    pcl::PointXYZ point(point_vec.x, point_vec.y, point_vec.z);
    edges_weak.push_back(point);
  }
  for (geometry_msgs::Vector3 point_vec : msg->lidar_surfaces_strong) {
    pcl::PointXYZ point(point_vec.x, point_vec.y, point_vec.z);
    surfaces_strong.push_back(point);
  }
  for (geometry_msgs::Vector3 point_vec : msg->lidar_surfaces_weak) {
    pcl::PointXYZ point(point_vec.x, point_vec.y, point_vec.z);
    surfaces_weak.push_back(point);
  }
  loam_cloud_ = beam_matching::LoamPointCloud(edges_strong, surfaces_strong,
                                              edges_weak, surfaces_weak);
}

std::vector<Eigen::Vector3d> ActiveSubmap::GetVisualMapPoints(
    const Eigen::Matrix4d& T_WORLD_CAMERA) {
  std::vector<Eigen::Vector3d> transformed_points;
  // transform each point and push to list
  for (auto& p : visual_map_points_) {
    Eigen::Vector4d ph{p[0], p[1], p[2], 1};
    Eigen::Vector3d tp = (T_WORLD_CAMERA.inverse() * ph).hnormalized();
    transformed_points.push_back(tp);
  }
  return transformed_points;
}

const std::vector<cv::Mat>& ActiveSubmap::GetDescriptors() {
  return descriptors_;
}

const pcl::PointCloud<pcl::PointXYZ> ActiveSubmap::GetPointCloud() {
  return point_cloud_;
}

void ActiveSubmap::RemoveVisualMapPoint(size_t index) {
  visual_map_points_.erase(visual_map_points_.begin() + index);
  descriptors_.erase(descriptors_.begin() + index);
}

}  // namespace bs_models