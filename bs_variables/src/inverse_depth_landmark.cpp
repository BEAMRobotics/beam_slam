#include <bs_variables/inverse_depth_landmark.h>

#include <ostream>

#include <boost/serialization/export.hpp>
#include <fuse_core/uuid.h>
#include <fuse_variables/fixed_size_variable.h>
#include <fuse_variables/stamped.h>
#include <pluginlib/class_list_macros.h>
#include <ros/time.h>

namespace bs_variables {

InverseDepthLandmark::InverseDepthLandmark(const uint64_t& id,
                                           const Eigen::Vector3d& bearing,
                                           const ros::Time& anchor_stamp)
    : FixedSizeVariable(fuse_core::uuid::generate(detail::type(), id)),
      id_(id),
      anchor_stamp_(anchor_stamp) {
  mx_ = bearing[0];
  my_ = bearing[1];
  if (bearing[2] != 1.0) {
    ROS_FATAL("Invalid bearing vector, 3rd component must equal 1.0.");
    throw std::runtime_error(
        "Invalid bearing vector, 3rd component must equal 1.0.");
  }
}

void InverseDepthLandmark::print(std::ostream& stream) const {
  stream << type() << ":\n"
         << "  uuid: " << uuid() << "\n"
         << "  landmark_id: " << id() << "\n"
         << "  anchor stamp: " << anchorStamp() << "\n"
         << "  size: " << size() << "\n"
         << "  data:\n"
         << "  - inverse_depth: " << inverse_depth() << "\n"
         << "  - mx: " << mx_ << "\n"
         << "  - my: " << my_ << "\n";
}

} // namespace bs_variables

BOOST_CLASS_EXPORT_IMPLEMENT(bs_variables::InverseDepthLandmark);
PLUGINLIB_EXPORT_CLASS(bs_variables::InverseDepthLandmark, fuse_core::Variable);