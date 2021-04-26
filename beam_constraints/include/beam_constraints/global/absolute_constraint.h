#pragma once

#include <fuse_core/constraint.h>
#include <fuse_core/eigen.h>
#include <fuse_core/macros.h>
#include <fuse_core/serialization.h>
#include <fuse_core/uuid.h>
#include <fuse_variables/acceleration_linear_3d_stamped.h>
#include <fuse_variables/velocity_angular_3d_stamped.h>
#include <fuse_variables/velocity_linear_3d_stamped.h>

#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <ceres/cost_function.h>

#include <beam_variables/imu_bias_stamped.h>

namespace beam_constraints { namespace global {

/**
 * @brief A constraint that represents prior information about a variable or a
 * direct measurement of the variable.
 *
 * This type of constraint arises in many situations. In mapping it is common to
 * define the very first pose as the origin. Some sensors, such as an IMU,
 * provide direct measurements of certain state variables (e.g. linear
 * acceleration). And localization systems often match laserscans to a prior map
 * (scan-to-map measurements). This constraint holds the measured variable value
 * and the measurement uncertainty/covariance.
 */
template <class Variable>
class AbsoluteConstraint3D : public fuse_core::Constraint {
public:
  FUSE_CONSTRAINT_DEFINITIONS(AbsoluteConstraint3D<Variable>);

  /**
   * @brief Default constructor
   */
  AbsoluteConstraint3D() = default;

  /**
   * @brief Create a constraint using a measurement/prior of all dimensions of
   * the target variable
   *
   * @param[in] source     The name of the sensor or motion model that generated
   * this constraint
   * @param[in] variable   An object derived from fuse_core::Variable.
   * @param[in] mean       The measured/prior value of all variable dimensions
   * @param[in] covariance The measurement/prior uncertainty of all variable
   * dimensions
   */
  AbsoluteConstraint3D(const std::string& source, const Variable& variable,
                       const fuse_core::VectorXd& mean,
                       const fuse_core::MatrixXd& covariance);

  /**
   * @brief Create a constraint using a measurement/prior of only a partial set
   * of dimensions of the target variable
   *
   * @param[in] source             The name of the sensor or motion model that
   * generated this constraint
   * @param[in] variable           An object derived from fuse_core::Variable.
   * @param[in] partial_mean       The measured value of the subset of
   * dimensions in the order defined by \p indices
   * @param[in] partial_covariance The uncertainty of the subset of dimensions
   * in the order defined by \p indices.
   * @param[in] indices            The set of indices corresponding to the
   * measured dimensions
   */
  AbsoluteConstraint3D(const std::string& source, const Variable& variable,
                       const fuse_core::VectorXd& partial_mean,
                       const fuse_core::MatrixXd& partial_covariance,
                       const std::vector<size_t>& indices);

  /**
   * @brief Destructor
   */
  virtual ~AbsoluteConstraint3D() = default;

  /**
   * @brief Read-only access to the measured/prior vector of mean values.
   *
   * All dimensions are present, even if only a partial set of dimensions were
   * measured. Dimensions are in the order defined by the variable, not the
   * order defined by the \p indices parameter. All unmeasured variable
   * dimensions are set to zero.
   */
  const fuse_core::VectorXd& mean() const { return mean_; }

  /**
   * @brief Read-only access to the square root information matrix.
   *
   * Dimensions are in the order defined by the variable, not the order defined
   * by the \p indices parameter. The square root information matrix will have
   * size measured_dimensions X variable_dimensions. If only a partial set of
   * dimensions are measured, then this matrix will not be square.
   */
  const fuse_core::MatrixXd& sqrtInformation() const {
    return sqrt_information_;
  }

  /**
   * @brief Compute the measurement covariance matrix.
   *
   * Dimensions are in the order defined by the variable, not the order defined
   * by the \p indices parameter. The covariance matrix will always be square,
   * with size variable_dimensions X variable_dimensions. If only a subset of
   * dimensions are measured, then some rows/columns will be zero. This will
   * result in a rank-deficient covariance matrix. You have been warned.
   */
  fuse_core::MatrixXd covariance() const;

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

protected:
  fuse_core::VectorXd
      mean_; //!< The measured/prior mean vector for this variable
  fuse_core::MatrixXd sqrt_information_; //!< The square root information matrix

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
    archive& mean_;
    archive& sqrt_information_;
  }
};

using AbsoluteVelocityAngular3DStampedConstraint =
    AbsoluteConstraint3D<fuse_variables::VelocityAngular3DStamped>;
using AbsoluteVelocityLinear3DStampedConstraint =
    AbsoluteConstraint3D<fuse_variables::VelocityLinear3DStamped>;
using AbsoluteAccelerationLinear3DStampedConstraint =
    AbsoluteConstraint3D<fuse_variables::AccelerationLinear3DStamped>;
using AbsoluteImuBias3DStampedConstraint =
    AbsoluteConstraint3D<beam_variables::ImuBiasStamped>;

}} // namespace beam_constraints::global

// Include the template implementation
#include <beam_constraints/global/absolute_constraint_impl.h>

BOOST_CLASS_EXPORT_KEY(
    beam_constraints::global::AbsoluteVelocityAngular3DStampedConstraint);
BOOST_CLASS_EXPORT_KEY(
    beam_constraints::global::AbsoluteVelocityLinear3DStampedConstraint);
BOOST_CLASS_EXPORT_KEY(
    beam_constraints::global::AbsoluteAccelerationLinear3DStampedConstraint);
BOOST_CLASS_EXPORT_KEY(
    beam_constraints::global::AbsoluteImuBias3DStampedConstraint);