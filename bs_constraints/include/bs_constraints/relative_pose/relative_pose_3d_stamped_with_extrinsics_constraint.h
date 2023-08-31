#pragma once

#include <fuse_core/constraint.h>
#include <fuse_core/eigen.h>
#include <fuse_core/fuse_macros.h>
#include <fuse_core/serialization.h>
#include <fuse_core/uuid.h>
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>

#include <Eigen/Dense>
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>

#include <ostream>
#include <string>
#include <vector>

#include <bs_variables/orientation_3d.h>
#include <bs_variables/position_3d.h>

namespace bs_constraints {

/**
 * @brief A constraint that represents a measurement on the difference between
 * two 3D poses.
 */
class RelativePose3DStampedWithExtrinsicsConstraint
    : public fuse_core::Constraint {
public:
  FUSE_CONSTRAINT_DEFINITIONS_WITH_EIGEN(
      RelativePose3DStampedWithExtrinsicsConstraint);

  /**
   * @brief Default constructor
   */
  RelativePose3DStampedWithExtrinsicsConstraint() = default;

  /**
   * @brief Constructor
   *
   * @param[in] source       The name of the sensor or motion model that
   * generated this constraint
   * @param[in] position1    The variable representing the position components
   * of the first pose
   * @param[in] orientation1 The variable representing the orientation
   * components of the first pose
   * @param[in] position2    The variable representing the position components
   * of the second pose
   * @param[in] orientation2 The variable representing the orientation
   * components of the second pose
   * @param[in] position_extrinsics The variable representing the position
   * components of the sensor frame relative to the baselink frame
   * @param[in] orientation_extrinsics The variable representing the orientation
   * components of the sensor frame relative to the baselink frame
   * @param[in] d_Sensor1_Sensor2 The exposed pose difference between pose 1 and
   * pose 2 in order (dx, dy, dz, dqw, dqx, dqy, dqz) expressed in the sensor
   * frame (e.g., lidar frame, camera frame)
   * @param[in] covariance   The measurement covariance (6x6 matrix: dx, dy, dz,
   * dqx, dqy, dqz)
   */
  RelativePose3DStampedWithExtrinsicsConstraint(
      const std::string& source,
      const fuse_variables::Position3DStamped& position1,
      const fuse_variables::Orientation3DStamped& orientation1,
      const fuse_variables::Position3DStamped& position2,
      const fuse_variables::Orientation3DStamped& orientation2,
      const bs_variables::Position3D& position_extrinsics,
      const bs_variables::Orientation3D& orientation_extrinsics,
      const fuse_core::Vector7d& d_Sensor1_Sensor2,
      const fuse_core::Matrix6d& covariance);

  /**
   * @brief Destructor
   */
  virtual ~RelativePose3DStampedWithExtrinsicsConstraint() = default;

  /**
   * @brief Read-only access to the measured pose change.
   */
  const fuse_core::Vector7d& delta() const { return d_Sensor1_Sensor2_; }

  /**
   * @brief Read-only access to the square root information matrix.
   */
  const fuse_core::Matrix6d& sqrtInformation() const {
    return sqrt_information_;
  }

  /**
   * @brief Compute the measurement covariance matrix.
   */
  fuse_core::Matrix6d covariance() const {
    return (sqrt_information_.transpose() * sqrt_information_).inverse();
  }

  /**
   * @brief Print a human-readable description of the constraint to the provided
   * stream.
   *
   * @param[out] stream The stream to write to. Defaults to stdout.
   */
  void print(std::ostream& stream = std::cout) const override;

  /**
   * @brief Access the cost function for this constraint
   *
   * The function caller will own the new cost function instance. It is the
   * responsibility of the caller to delete the cost function object when it is
   * no longer needed. If the pointer is provided to a Ceres::Problem object,
   * the Ceres::Problem object will takes ownership of the pointer and delete it
   * during destruction.
   *
   * @return A base pointer to an instance of a derived CostFunction.
   */
  ceres::CostFunction* costFunction() const override;

protected:
  /** The measured difference between variable pose2 and variable pose1 in the
   * sensor frame. Note we invert the input difference here instead of inverting
   * the current estimate of the pose at each iteration of the optimizer */
  fuse_core::Vector7d d_Sensor1_Sensor2_;

  /** The square root information matrix used as the residual weighting matrix
   */
  fuse_core::Matrix6d sqrt_information_;

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
    archive& boost::serialization::base_object<fuse_core::Constraint>(*this);
    archive& d_Sensor1_Sensor2_;
    archive& sqrt_information_;
  }
};

} // namespace bs_constraints

BOOST_CLASS_EXPORT_KEY(
    bs_constraints::RelativePose3DStampedWithExtrinsicsConstraint);
