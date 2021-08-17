#include <bs_models/camera_to_camera/visual_submap.h>

#include <beam_cv/descriptors/Descriptor.h>

namespace bs_models { namespace camera_to_camera {

VisualSubmap::VisualSubmap() {}

void VisualSubmap::SetSubmap(const SubmapMsg::ConstPtr& msg) {
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
}

std::vector<Eigen::Vector3d>
    VisualSubmap::GetVisualMapPoints(const Eigen::Matrix4d& T_WORLD_CAMERA) {
  std::vector<Eigen::Vector3d> transformed_points;
  // transform each point and push to list
  for (auto& p : visual_map_points_) {
    Eigen::Vector4d ph{p[0], p[1], p[2], 1};
    Eigen::Vector3d tp = (T_WORLD_CAMERA.inverse() * ph).hnormalized();
    transformed_points.push_back(tp);
  }
  return transformed_points;
}

const std::vector<cv::Mat>& VisualSubmap::GetDescriptors() {
  return descriptors_;
}

void VisualSubmap::RemoveVisualMapPoint(size_t index) {
  visual_map_points_.erase(visual_map_points_.begin() + index);
  descriptors_.erase(descriptors_.begin() + index);
}

}} // namespace bs_models::camera_to_camera