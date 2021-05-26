#include <beam_variables/accel_bias_3d_stamped.h>

#include <ostream>

#include <boost/serialization/export.hpp>
#include <fuse_core/uuid.h>
#include <fuse_variables/fixed_size_variable.h>
#include <fuse_variables/stamped.h>
#include <pluginlib/class_list_macros.h>
#include <ros/time.h>

namespace beam_variables {

AccelerationBias3DStamped::AccelerationBias3DStamped(
    const ros::Time& stamp, const fuse_core::UUID& device_id)
    : FixedSizeVariable(
          fuse_core::uuid::generate(detail::type(), stamp, device_id)),
      Stamped(stamp, device_id) {}

void AccelerationBias3DStamped::print(std::ostream& stream) const {
  stream << type() << ":\n"
         << "  uuid: " << uuid() << "\n"
         << "  device_id: " << deviceId() << "\n"
         << "  stamp: " << stamp() << "\n"
         << "  size: " << size() << "\n"
         << "  data:\n"
         << "  - x: " << x() << "\n"
         << "  - y: " << y() << "\n"
         << "  - z: " << z() << "\n";
}

}  // namespace beam_variables

BOOST_CLASS_EXPORT_IMPLEMENT(beam_variables::AccelerationBias3DStamped);
PLUGINLIB_EXPORT_CLASS(beam_variables::AccelerationBias3DStamped,
                       fuse_core::Variable);