#pragma once

#include <ostream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <fuse_core/constraint.h>
#include <fuse_core/eigen.h>
#include <fuse_core/fuse_macros.h>
#include <fuse_core/serialization.h>
#include <fuse_core/uuid.h>

namespace bs_constraints { namespace global {

/**
 * @brief A constraint that aligns some pose to the absolute gravity direction
 * measured by another source. IMUs are good at estimating absolute orientation
 * where roll and pitch are well estimated by using the gravity vector, usually
 * by assuming net zero acceleration. A lo of IMUs will run filters to estimate
 * absolute orientation. You can directly use that with this constraint to keep
 * poses gravity locked
 */
class GravityAlignmentStampedConstraint : public fuse_core::Constraint {
public:
  FUSE_CONSTRAINT_DEFINITIONS_WITH_EIGEN(GravityAlignmentStampedConstraint);

  /**
   * @brief Default constructor
   */
  GravityAlignmentStampedConstraint() = default;

  /**
   * @brief Create a constraint using a measurement of orientation wrt IMU frame
   * @param source the name of the sensor or motion model that generated this
   * constraint
   * @param orientation_uuid uid of the orientation variable to constrain
   * @param gravity_in_baselink measured gravity in the baselink frame
   * @param covariance the measurement covariance (2x2 matrix: ex, ey)
   */
  GravityAlignmentStampedConstraint(
      const std::string& source, const fuse_core::UUID& orientation_uuid,
      const Eigen::Vector3d& gravity_in_baselink,
      const Eigen::Matrix<double, 2, 2>& covariance);

  /**
   * @brief Destructor
   */
  virtual ~GravityAlignmentStampedConstraint() = default;

  /**
   * @brief Read-only access to the measured orientation vector
   */
  const Eigen::Vector3d& gravity_in_baselink() const {
    return gravity_in_baselink_;
  }

  /**
   * @brief Read-only access to the square root information matrix.
   */
  const Eigen::Matrix<double, 2, 2>& sqrtInformation() const {
    return sqrt_information_;
  }

  /**
   * @brief Compute the measurement covariance matrix.
   */
  Eigen::Matrix<double, 2, 2> covariance() const {
    return (sqrt_information_.transpose() * sqrt_information_).inverse();
  }

  /**
   * @brief Print a human-readable description of the constraint to the provided
   * stream.
   * @param stream the stream to write to. Defaults to stdout.
   */
  void print(std::ostream& stream = std::cout) const override;

  /**
   * @brief Construct an instance of this constraint's cost function
   *
   * The function caller will own the new cost function instance. It is the
   * responsibility of the caller to delete the cost function object when it is
   * no longer needed. If the pointer is provided to a Ceres::Problem object,
   * the Ceres::Problem object will takes ownership of the pointer and delete it
   * during destruction.
   *
   * @return a base pointer to an instance of a derived CostFunction.
   */
  ceres::CostFunction* costFunction() const override;

protected:
  Eigen::Vector3d gravity_in_baselink_;
  Eigen::Matrix<double, 2, 2> sqrt_information_;

private:
  // Allow Boost Serialization access to private methods
  friend class boost::serialization::access;

  /**
   * @brief The Boost Serialize method that serializes all of the data members
   * in to/out of the archive
   * @param archive - The archive object that holds the serialized class members
   * @param version - The version of the archive being read/written. Generally
   * unused.
   */
  template <class Archive>
  void serialize(Archive& archive, const unsigned int /* version */) {
    archive& boost::serialization::base_object<fuse_core::Constraint>(*this);
    archive& gravity_in_baselink_;
    archive& sqrt_information_;
  }
};

}} // namespace bs_constraints::global

BOOST_CLASS_EXPORT_KEY(
    bs_constraints::global::GravityAlignmentStampedConstraint);
