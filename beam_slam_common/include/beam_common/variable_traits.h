#pragma once

#include <fuse_variables/acceleration_linear_3d_stamped.h>
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>
#include <fuse_variables/velocity_angular_3d_stamped.h>
#include <fuse_variables/velocity_linear_3d_stamped.h>

namespace beam_common {

template <typename T>
struct is_linear_3d {
  static const bool value = false;
};

template <>
struct is_linear_3d<fuse_variables::AccelerationLinear3DStamped> {
  static const bool value = true;
};

template <>
struct is_linear_3d<fuse_variables::VelocityLinear3DStamped> {
  static const bool value = true;
};

template <>
struct is_linear_3d<fuse_variables::Position3DStamped> {
  static const bool value = true;
};

template <typename T>
struct is_angular_3d {
  static const bool value = false;
};

template <>
struct is_angular_3d<fuse_variables::Orientation3DStamped> {
  static const bool value = true;
};

template <>
struct is_angular_3d<fuse_variables::VelocityAngular3DStamped> {
  static const bool value = true;
};

} // namespace beam_common
