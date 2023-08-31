#pragma once

#include <fuse_core/fuse_macros.h>
#include <fuse_core/serialization.h>
#include <fuse_core/util.h>
#include <fuse_variables/fixed_size_variable.h>
#include <fuse_variables/orientation_3d_stamped.h>

#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>

#include <ostream>

namespace bs_variables {

/**
 * @brief Variable representing a 3D orientation that is time invariant. A
 * common use case for this type of variable is estimating the extrinsic
 * calibrations of a device. Since there can be many static orientations being
 * estimated which represent different sensor frames, we require two frame ids
 * to instantiate this class in order to fully and uniquely describe the
 * variable.
 */
class Orientation3D : public fuse_variables::FixedSizeVariable<4> {
public:
  FUSE_VARIABLE_DEFINITIONS(Orientation3D);

  /**
   * @brief Can be used to directly index variables in the data array
   */
  enum : size_t { W = 0, X = 1, Y = 2, Z = 3 };

  /**
   * @brief default constructor is required for the macros but should not be
   * used to to construct of instance of this class
   */
  Orientation3D() = default;

  /**
   * @brief Construct a position 3D variable given two frame ids. Note that ROS
   * uses the terms child and parent to express the 'from' to 'to' frames,
   * respectively. In other words, Pose = T_To_From = T_child_parent. For
   * example, T_World_Baselink (child is world and parent is baselink), or
   * T_Baselink_Sensor (child is baselink and parent is sensor)
   *
   * @param[in] child_frame  id of the child frame
   * @param[in] parent_frame  id of the parent frame
   */
  explicit Orientation3D(const std::string& child_frame,
                         const std::string& parent_frame);

  /**
   * @brief Read-write access to the W value of the quaternion.
   */
  double& w() { return data_[W]; }

  /**
   * @brief Read-only access to the W value of the quaternion.
   */
  const double& w() const { return data_[W]; }

  /**
   * @brief Read-write access to the X value of the quaternion.
   */
  double& x() { return data_[X]; }

  /**
   * @brief Read-only access to the X value of the quaternion.
   */
  const double& x() const { return data_[X]; }

  /**
   * @brief Read-write access to the Y value of the quaternion.
   */
  double& y() { return data_[Y]; }

  /**
   * @brief Read-only access to the Y value of the quaternion.
   */
  const double& y() const { return data_[Y]; }

  /**
   * @brief Read-write access to the Z value of the quaternion.
   */
  double& z() { return data_[Z]; }

  /**
   * @brief Read-only access to the Z value of the quaternion.
   */
  const double& z() const { return data_[Z]; }

  /**
   * @brief Read-only access to the child frame id
   */
  const std::string& child() const { return child_frame_; }

  /**
   * @brief Read-only access to the parent frame id
   */
  const std::string& parent() const { return parent_frame_; }

  /**
   * @brief Read-only access to quaternion's Euler roll angle component
   */
  double roll() { return fuse_core::getRoll(w(), x(), y(), z()); }

  /**
   * @brief Read-only access to quaternion's Euler pitch angle component
   */
  double pitch() { return fuse_core::getPitch(w(), x(), y(), z()); }

  /**
   * @brief Read-only access to quaternion's Euler yaw angle component
   */
  double yaw() { return fuse_core::getYaw(w(), x(), y(), z()); }

  /**
   * @brief Print a human-readable description of the variable to the provided
   * stream.
   *
   * @param[out] stream The stream to write to. Defaults to stdout.
   */
  void print(std::ostream& stream = std::cout) const override;

  /**
   * @brief Provides a Ceres local parameterization for the quaternion
   *
   * @return A pointer to a local parameterization object that indicates how to
   * "add" increments to the quaternion
   */
  fuse_core::LocalParameterization* localParameterization() const override;

  /**
   * @brief Specifies if the value of the variable should not be changed during
   * optimization
   */
  bool holdConstant() const override;

private:
  // Allow Boost Serialization access to private methods
  friend class boost::serialization::access;
  std::string parent_frame_;
  std::string child_frame_;

  /**
   * @brief The Boost Serialize method that serializes all of the data members
   * in to/out of the archive
   *
   * @param[in/out] archive - The archive object that holds the serialized class
   * members
   * @param[in] version - The version of the archive being read/written.
   * Generally unused.
   */
  template <class Archive>
  void serialize(Archive& archive, const unsigned int /* version */) {
    archive& boost::serialization::base_object<FixedSizeVariable<SIZE>>(*this);
    archive& parent_frame_;
    archive& child_frame_;
  }
};

} // namespace bs_variables

BOOST_CLASS_EXPORT_KEY(bs_variables::Orientation3D);
