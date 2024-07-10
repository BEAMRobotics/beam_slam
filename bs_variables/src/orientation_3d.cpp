#include <bs_variables/orientation_3d.h>

#include <fuse_core/local_parameterization.h>
#include <fuse_core/uuid.h>
#include <fuse_core/variable.h>
#include <fuse_variables/fixed_size_variable.h>
#include <pluginlib/class_list_macros.h>

#include <boost/serialization/export.hpp>

#include <ostream>

namespace bs_variables {

Orientation3D::Orientation3D(const std::string& child_frame,
                             const std::string& parent_frame)
    : FixedSizeVariable(fuse_core::uuid::generate(detail::type(),
                                                  child_frame + parent_frame)),
      parent_frame_(parent_frame),
      child_frame_(child_frame) {}

void Orientation3D::print(std::ostream& stream) const {
  stream << type() << ":\n"
         << "  uuid: " << uuid() << "\n"
         << "  size: " << size() << "\n"
         << "  parent frame: " << parent() << "\n"
         << "  child frame: " << child() << "\n"
         << "  data:\n"
         << "  - w: " << w() << "\n"
         << "  - x: " << x() << "\n"
         << "  - y: " << y() << "\n"
         << "  - z: " << z() << "\n";
}

fuse_core::LocalParameterization* Orientation3D::localParameterization() const {
  return new fuse_variables::Orientation3DLocalParameterization();
}

bool Orientation3D::holdConstant() const {
  return false;
}

} // namespace bs_variables

BOOST_CLASS_EXPORT_IMPLEMENT(bs_variables::Orientation3D);
PLUGINLIB_EXPORT_CLASS(bs_variables::Orientation3D, fuse_core::Variable);
