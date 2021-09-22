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
  point_cloud_.points.clear();
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
    for (int i = 0; i < msg->lidar_map.size() - 2; i += 3) {
      pcl::PointXYZ point(msg->lidar_map[i], msg->lidar_map[i + 1],
                          msg->lidar_map[i + 2]);
      point_cloud_.points.push_back(point);
    }
  }
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