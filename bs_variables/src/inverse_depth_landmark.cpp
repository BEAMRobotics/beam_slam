#include <bs_variables/gyro_bias_3d_stamped.h>

#include <ostream>

#include <boost/serialization/export.hpp>
#include <fuse_core/uuid.h>
#include <fuse_variables/fixed_size_variable.h>
#include <fuse_variables/stamped.h>
#include <pluginlib/class_list_macros.h>
#include <ros/time.h>

namespace bs_variables {

InverseDepthLandmark::InverseDepthLandmark(const uint64_t& id,
                                           const ros::Time& anchor_time)
    : FixedSizeVariable(fuse_core::uuid::generate(detail::type(), id)),
      id_(id),
      anchor_stamp_(anchor_time) {}

void InverseDepthLandmark::print(std::ostream& stream) const {
  stream << type() << ":\n"
         << "  uuid: " << uuid() << "\n"
         << "  landmark_id: " << id() << "\n"
         << "  anchor stamp: " << anchorStamp() << "\n"
         << "  size: " << size() << "\n"
         << "  data:\n"
         << "  - rho: " << rho() << "\n"
         << "  - mx: " << bearing()[0] << "\n"
         << "  - my: " << bearing()[1] << "\n";
}

} // namespace bs_variables

BOOST_CLASS_EXPORT_IMPLEMENT(bs_variables::InverseDepthLandmark);
PLUGINLIB_EXPORT_CLASS(bs_variables::InverseDepthLandmark, fuse_core::Variable);