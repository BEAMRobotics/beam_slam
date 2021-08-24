#pragma once

#include <ostream>
#include <string>
#include <vector>

#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <Eigen/Dense>
#include <fuse_core/constraint.h>
#include <fuse_core/eigen.h>
#include <fuse_core/macros.h>
#include <fuse_core/serialization.h>
#include <fuse_core/uuid.h>

#include <bs_common/imu_state.h>

namespace bs_constraints {
namespace global {

/**
 * @brief A constraint that represents either prior information about an IMU
 * state, or a direct measurement of the IMU state. This constaint is typically
 * used after IMU initialization, when an estimate of the first IMU pose (along
 * with velocity and bias estimates) is made.
 *
 * This constraint holds the measured IMU state and the measurement
 * uncertainty/covariance. Orientations are represented as quaternions.
 */
class AbsoluteImuState3DStampedConstraint : public fuse_core::Constraint {
 public:
  FUSE_CONSTRAINT_DEFINITIONS_WITH_EIGEN(AbsoluteImuState3DStampedConstraint);

  /**
   * @brief Default constructor
   */
  AbsoluteImuState3DStampedConstraint() = default;

  /**
   * @brief Create a constraint using a measurement/prior of the IMU state
   * @param source the name of the sensor or motion model that generated this
   * constraint
   * @param imu_state_i the measured/prior IMU state
   * @param mean the measured/prior IMU state as a vector (16x1 vector: q, p, v,
   * bg, ba)
   * @param covariance the measurement/prior covariance (15x15 matrix: q, p, v,
   * bg, ba)
   */
  AbsoluteImuState3DStampedConstraint(
      const std::string& source, const bs_common::ImuState& imu_state,
      const Eigen::Matrix<double, 16, 1>& mean,
      const Eigen::Matrix<double, 15, 15>& covariance);

  /**
   * @brief Destructor
   */
  virtual ~AbsoluteImuState3DStampedConstraint() = default;

  /**
   * @brief Read-only access to the measured/prior vector of mean values.
   * Order is (q, p, v, bg, ba)
   */
  const Eigen::Matrix<double, 16, 1>& mean() const { return mean_; }

  /**
   * @brief Read-only access to the square root information matrix.
   * Order is (q, p, v, bg, ba)
   */
  const Eigen::Matrix<double, 15, 15>& sqrtInformation() const {
    return sqrt_information_;
  }

  /**
   * @brief Compute the measurement covariance matrix.
   * Order is (q, p, v, bg, ba)
   */
  Eigen::Matrix<double, 15, 15> covariance() const {
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
  Eigen::Matrix<double, 16, 1> mean_;
  Eigen::Matrix<double, 15, 15> sqrt_information_;

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
    archive& mean_;
    archive& sqrt_information_;
  }
};

}  // namespace global
}  // namespace bs_constraints

BOOST_CLASS_EXPORT_KEY(
    bs_constraints::global::AbsoluteImuState3DStampedConstraint);
