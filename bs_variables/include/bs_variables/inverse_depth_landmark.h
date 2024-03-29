#pragma once

#include <ostream>

#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <fuse_core/serialization.h>
#include <fuse_core/uuid.h>
#include <fuse_core/variable.h>
#include <fuse_variables/fixed_size_variable.h>
#include <ros/time.h>

namespace bs_variables {

/**
 * @brief Variable representing an Inverse depth landmark. An Inverse depth
 * landmark is defined by the inverse depth (1/d) the landmark is from an
 * "Anchor" pose. It is also defined by the bearing vector from that pose
 * (T_WORLD_CAMERA), however only the inverse depth is optimized.
 */
class InverseDepthLandmark : public fuse_variables::FixedSizeVariable<1> {
public:
  FUSE_VARIABLE_DEFINITIONS(InverseDepthLandmark);

  /**
   * @brief Default constructor
   */
  InverseDepthLandmark() = default;

  /**
   * @brief Construct an inverse depth landmark.
   *
   * @param[in] id of landmark
   * @param[in] bearing unit bearing vector from anchor pose in the camera frame
   * @param[in] anchor_stamp timestamp of the anchor pose to this landmark
   */
  explicit InverseDepthLandmark(const uint64_t& id,
                                const Eigen::Vector3d& bearing,
                                const ros::Time& anchor_stamp);

  /**
   * @brief Read-write access to the inverse depth
   */
  double& inverse_depth() { return data_[0]; }

  /**
   * @brief Read-only access to the inverse depth
   */
  const double& inverse_depth() const { return data_[0]; }

  /**
   * @brief Access to the point in the anchor camera frame
   */
  Eigen::Vector3d camera_t_point() const {
    double depth = 1 / data_[0];
    Eigen::Vector3d point = depth * bearing();
    return point;
  }

  /**
   * @brief Access to the bearing vector from the anchor frame
   */
  Eigen::Vector3d bearing() const { return bearing_; }

  /**
   * @brief Read-only access to the id
   */
  const uint64_t& id() const { return id_; }

  /**
   * @brief Read-only access to the id
   */
  const ros::Time& anchorStamp() const { return anchor_stamp_; }

  /**
   * @brief Print a human-readable description of the variable to the provided
   * stream.
   *
   * @param  stream The stream to write to. Defaults to stdout.
   */
  void print(std::ostream& stream = std::cout) const override;

private:
  Eigen::Vector3d bearing_;
  uint64_t id_{0};
  ros::Time anchor_stamp_;

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
  }
};

} // namespace bs_variables

BOOST_CLASS_EXPORT_KEY(bs_variables::InverseDepthLandmark);
