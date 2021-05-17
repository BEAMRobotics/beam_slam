#pragma once

#include <ostream>

#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <fuse_core/uuid.h>
#include <fuse_core/serialization.h>
#include <fuse_core/variable.h>
#include <fuse_variables/fixed_size_variable.h>
#include <fuse_variables/stamped.h>
#include <ros/time.h>

namespace beam_variables {

/**
 * @brief Variable representing IMU acceleration bias (Bax, Bay, Baz) at a
 * specific time and for a specific piece of hardware (e.g., robot)
 */
class ImuBiasAccel3DStamped : public fuse_variables::FixedSizeVariable<3>,
                             public fuse_variables::Stamped {
public:
  FUSE_VARIABLE_DEFINITIONS(ImuBiasAccel3DStamped);

  /**
   * @brief Can be used to directly index variables in the data array
   */
  enum : size_t { X = 0, Y = 1, Z = 2 };

  /**
   * @brief Default constructor
   */
  ImuBiasAccel3DStamped() = default;

  /**
   * @brief Construct an IMU bias at a specific point in time.
   *
   * @param[in] stamp The timestamp attached to this imu bias.
   * @param[in] device_id An optional device id, for use when variables
   * originate from multiple robots or devices
   */
  explicit ImuBiasAccel3DStamped(
      const ros::Time& stamp,
      const fuse_core::UUID& device_id = fuse_core::uuid::NIL);

  /**
   * @brief Read-write access to the X-axis bias.
   */
  double& x() { return data_[X]; }

  /**
   * @brief Read-only access to the X-axis bias.
   */
  const double& x() const { return data_[X]; }

  /**
   * @brief Read-write access to the Y-axis bias.
   */
  double& y() { return data_[Y]; }

  /**
   * @brief Read-only access to the Y-axis bias.
   */
  const double& y() const { return data_[Y]; }

  /**
   * @brief Read-write access to the Z-axis bias.
   */
  double& z() { return data_[Z]; }

  /**
   * @brief Read-only access to the Z-axis bias.
   */
  const double& z() const { return data_[Z]; }

  /**
   * @brief Print a human-readable description of the variable to the provided
   * stream.
   *
   * @param  stream The stream to write to. Defaults to stdout.
   */
  void print(std::ostream& stream = std::cout) const override;

private:
  // Allow Boost Serialization access to private methods
  friend class boost::serialization::access;

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
    archive& boost::serialization::base_object<Stamped>(*this);
  }
};

}  // namespace beam_variables

BOOST_CLASS_EXPORT_KEY(beam_variables::ImuBiasAccel3DStamped);
