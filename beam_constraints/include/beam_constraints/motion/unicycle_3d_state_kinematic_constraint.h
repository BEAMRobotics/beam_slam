#pragma once

#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <fuse_core/constraint.h>
#include <fuse_core/eigen.h>
#include <fuse_core/macros.h>
#include <fuse_core/serialization.h>
#include <fuse_variables/acceleration_linear_3d_stamped.h>
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>
#include <fuse_variables/velocity_angular_3d_stamped.h>
#include <fuse_variables/velocity_linear_3d_stamped.h>

namespace fuse_core {
using Matrix15d = Eigen::Matrix<double, 15, 15, Eigen::RowMajor>;
using Vector15d = Eigen::Matrix<double, 15, 1>;

} // namespace fuse_core

namespace beam_constraints { namespace motion {

/**
 * @brief A class that represents a kinematic constraint between 3D states at
 * two different times
 *
 * The fuse_models 3D state is a combination of 3D position, 3D orientation, 3D
 * linear velocity, 3D angular velocity, and 3D linear acceleration.
 */
class Unicycle3DStateKinematicConstraint : public fuse_core::Constraint {
public:
  FUSE_CONSTRAINT_DEFINITIONS_WITH_EIGEN(Unicycle3DStateKinematicConstraint);

  /**
   * @brief Default constructor
   */
  Unicycle3DStateKinematicConstraint() = default;

  /**
   * @brief Create a constraint using a time delta and a kinematic model cost
   * functor
   *
   * The constraint is created between two states. The state is broken up into
   * multiple fuse variable types.
   *
   * @param[in] source The name of the sensor or motion model that generated
   * this constraint
   * @param[in] position1 Position component variable of the fist state
   * @param[in] orientation1 Yaw component variable of the first state
   * @param[in] linear_velocity1 Linear velocity component variable of the first
   * state
   * @param[in] angular_velocity1 Yaw velocity component variable of the first
   * state
   * @param[in] linear_acceleration1 Linear acceleration component variable of
   * the first state
   * @param[in] position2 Position component variable of the second state
   * @param[in] orientation2 Yaw component variable of the second state
   * @param[in] linear_velocity2 Linear velocity component variable of the
   * second state
   * @param[in] angular_velocity2 Yaw velocity component variable of the second
   * state
   * @param[in] linear_acceleration2 Linear acceleration component variable of
   * the second state
   * @param[in] covariance - The covariance matrix used to weight the
   * constraint. Order is (x, y, yaw, x_vel, y_vel, yaw_vel, x_acc, y_acc)
   */
  Unicycle3DStateKinematicConstraint(
      const std::string& source,
      const fuse_variables::Position3DStamped& position1,
      const fuse_variables::Orientation3DStamped& orientation1,
      const fuse_variables::VelocityLinear3DStamped& linear_velocity1,
      const fuse_variables::VelocityAngular3DStamped& angular_velocity1,
      const fuse_variables::AccelerationLinear3DStamped& linear_acceleration1,
      const fuse_variables::Position3DStamped& position2,
      const fuse_variables::Orientation3DStamped& orientation2,
      const fuse_variables::VelocityLinear3DStamped& linear_velocity2,
      const fuse_variables::VelocityAngular3DStamped& angular_velocity2,
      const fuse_variables::AccelerationLinear3DStamped& linear_acceleration2,
      const fuse_core::Matrix15d& covariance);

  /**
   * @brief Destructor
   */
  virtual ~Unicycle3DStateKinematicConstraint() = default;

  /**
   * @brief Read-only access to the time delta between the first and second
   state (really, between the position1 and position2 variables in the
   constructor)
   */
  double dt() const { return dt_; }

  /**
   * @brief Read-only access to the square root information matrix.
   *
   * Order is (x, y, yaw, x_vel, y_vel, yaw_vel, x_acc, y_acc)
   */
  const fuse_core::Matrix15d& sqrtInformation() const {
    return sqrt_information_;
  }

  /**
   * @brief Compute the measurement covariance matrix.
   *
   * Order is (x, y, yaw, x_vel, y_vel, yaw_vel, x_acc, y_acc)
   */
  fuse_core::Matrix15d covariance() const {
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

protected:
  double dt_; //!< The time delta for the constraint
  fuse_core::Matrix15d
      sqrt_information_; //!< The square root information matrix

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
    archive& dt_;
    archive& sqrt_information_;
  }
};

}} // namespace beam_constraints::motion

BOOST_CLASS_EXPORT_KEY(beam_constraints::motion::Unicycle3DStateKinematicConstraint);
