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

#include <bs_common/imu_state.h>
#include <bs_common/preintegrator.h>

namespace bs_constraints {

/**
 * @brief A constraint that represents a measurement on the difference between
 * IMU states using the methods outlined in DOI: 10.15607/RSS.2015.XI.006
 *
 * This constraint holds both IMU states and their associated preintegrator to
 * allow for fast updating of predicted states if bias estimates change during
 * optimization
 */
class RelativeImuState3DStampedConstraint : public fuse_core::Constraint {
public:
  FUSE_CONSTRAINT_DEFINITIONS_WITH_EIGEN(RelativeImuState3DStampedConstraint);

  /**
   * @brief Default constructor
   */
  RelativeImuState3DStampedConstraint() = default;

  /**
   * @brief Constructor
   * @param source the name of the sensor or motion model that generated
   * this constraint
   * @param imu_state_i the current IMU state
   * @param imu_state_j the new IMU state
   * @param pre_integrator preintegrator class containing IMU data between new
   * and current IMU states
   */
  RelativeImuState3DStampedConstraint(
      const std::string& source, const bs_common::ImuState& imu_state_i,
      const bs_common::ImuState& imu_state_j,
      const std::shared_ptr<bs_common::PreIntegrator>& pre_integrator,
      const double info_weight);

  /**
   * @brief Destructor
   */
  virtual ~RelativeImuState3DStampedConstraint() = default;

  /**
   * @brief Print a human-readable description of the constraint to the provided
   * stream.
   * @param stream the stream to write to. Defaults to stdout.
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
   * @return a base pointer to an instance of a derived CostFunction.
   */
  ceres::CostFunction* costFunction() const override;

  /**
   * @brief get relative pose between two IMU states
   */
  Eigen::Matrix4d getRelativePose() const;

  /**
   * @brief get time of first state
   */
  ros::Time GetTime1() { return imu_state_i_.Stamp(); }

  /**
   * @brief get time of second state
   */
  ros::Time GetTime2() { return imu_state_j_.Stamp(); }

protected:
  bs_common::ImuState imu_state_i_;
  bs_common::ImuState imu_state_j_;
  std::shared_ptr<bs_common::PreIntegrator> pre_integrator_;
  double info_weight_;

private:
  // Allow Boost Serialization access to private methods
  friend class boost::serialization::access;

  /**
   * @brief The Boost Serialize method that serializes all of the data members
   * in to/out of the archive
   * @param archive the archive object that holds the serialized class members
   * @param version the version of the archive being read/written. Generally
   * unused.
   */
  template <class Archive>
  void serialize(Archive& archive, const unsigned int /* version */) {
    archive& boost::serialization::base_object<fuse_core::Constraint>(*this);
  }
};

} // namespace bs_constraints

BOOST_CLASS_EXPORT_KEY(bs_constraints::RelativeImuState3DStampedConstraint);
