#pragma once

#include <Eigen/Dense>
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <fuse_core/constraint.h>
#include <fuse_core/eigen.h>
#include <fuse_core/fuse_macros.h>
#include <fuse_core/serialization.h>
#include <fuse_core/uuid.h>

#include <bs_variables/orientation_3d.h>
#include <bs_variables/position_3d.h>

namespace bs_constraints {

/**
 * @brief A constraint that represents either prior information about a 3D pose,
 * or a direct measurement of the 3D pose. Note that this is the same
 * implementation as fuse_constraints/absolute_pose_3d_stamped_constraint
 */
class AbsolutePose3DConstraint : public fuse_core::Constraint {
public:
  FUSE_CONSTRAINT_DEFINITIONS_WITH_EIGEN(AbsolutePose3DConstraint);

  /**
   * @brief Default constructor
   */
  AbsolutePose3DConstraint() = default;

  /**
   * @brief Create a constraint using a measurement/prior of the 3D pose
   *
   * @param[in] source      The name of the sensor or motion model that
   * generated this constraint
   * @param[in] position    The variable representing the position components of
   * the pose
   * @param[in] orientation The variable representing the orientation components
   * of the pose
   * @param[in] mean        The measured/prior pose as a vector (7x1 vector: x,
   * y, z, qw, qx, qy, qz)
   * @param[in] covariance  The measurement/prior covariance (6x6 matrix: x, y,
   * z, qx, qy, qz)
   */
  AbsolutePose3DConstraint(const std::string& source,
                           const bs_variables::Position3D& position,
                           const bs_variables::Orientation3D& orientation,
                           const fuse_core::Vector7d& mean,
                           const fuse_core::Matrix6d& covariance);

  /**
   * @brief Destructor
   */
  virtual ~AbsolutePose3DConstraint() = default;

  /**
   * @brief Read-only access to the measured/prior vector of mean values.
   *
   * Order is (x, y, z, qw, qx, qy, qz)
   */
  const fuse_core::Vector7d& mean() const { return mean_; }

  /**
   * @brief Read-only access to the square root information matrix.
   *
   * Order is (x, y, z, qx, qy, qz)
   */
  const fuse_core::Matrix6d& sqrtInformation() const {
    return sqrt_information_;
  }

  /**
   * @brief Compute the measurement covariance matrix.
   *
   * Order is (x, y, z, qx, qy, qz)
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
   * @brief Construct an instance of this constraint's cost function
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

  bs_variables::Position3D getInitialPosition();

  bs_variables::Orientation3D getInitialOrientation();

protected:
  fuse_core::Vector7d
      mean_; //!< The measured/prior mean vector for this variable
  fuse_core::Matrix6d sqrt_information_; //!< The square root information matrix
  bs_variables::Position3D position_;    // initial input position
  bs_variables::Orientation3D orientation_; // initial input orientation

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
    archive & mean_;
    archive & sqrt_information_;
  }
};

} // namespace bs_constraints

BOOST_CLASS_EXPORT_KEY(bs_constraints::AbsolutePose3DConstraint);
