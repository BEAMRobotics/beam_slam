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
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>
#include <fuse_variables/velocity_linear_3d_stamped.h>

#include <beam_variables/imu_bias_gyro_3d_stamped.h>
#include <beam_variables/imu_bias_accel_3d_stamped.h>
#include <slamtools/preintegrator.h>

namespace beam_constraints { namespace frame_to_frame {

/**
 * @brief A constraint that represents a measurement on the difference between
 * imu states
 *
 * This constraint holds the measured 3D imu state change as well as the
 * measurement uncertainty/covariance.
 */
class RelativeImuState3DStampedConstraint : public fuse_core::Constraint {
public:
  FUSE_CONSTRAINT_DEFINITIONS_WITH_EIGEN(RelativeImuState3DStampedConstraint);

  RelativeImuState3DStampedConstraint() = default;

  RelativeImuState3DStampedConstraint(
      const std::string& source,
      const fuse_variables::Orientation3DStamped& orientation1,
      const fuse_variables::Position3DStamped& position1,
      const fuse_variables::VelocityLinear3DStamped& velocity1,
      const beam_variables::ImuBiasGyro3DStamped& gyrobias1,
      const beam_variables::ImuBiasAccel3DStamped& accelbias1,
      const fuse_variables::Orientation3DStamped& orientation2,
      const fuse_variables::Position3DStamped& position2,
      const fuse_variables::VelocityLinear3DStamped& velocity2,
      const beam_variables::ImuBiasGyro3DStamped& gyrobias2,
      const beam_variables::ImuBiasAccel3DStamped& accelbias2,
      const Eigen::Matrix<double, 16, 1>& delta,
      const Eigen::Matrix<double, 15, 15>& covariance,
      const std::shared_ptr<PreIntegrator> pre_integrator = nullptr);

  virtual ~RelativeImuState3DStampedConstraint() = default;

  const Eigen::Matrix<double, 16, 1>& delta() const { return delta_; }

  const Eigen::Matrix<double, 15, 15>& sqrtInformation() const {
    return sqrt_information_;
  }

  Eigen::Matrix<double, 15, 15> covariance() const {
    return (sqrt_information_.transpose() * sqrt_information_).inverse();
  }

  void print(std::ostream& stream = std::cout) const override;

  ceres::CostFunction* costFunction() const override;

protected:
  Eigen::Matrix<double, 16, 1> delta_;
  Eigen::Matrix<double, 15, 15> sqrt_information_;

private:
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& archive, const unsigned int /* version */) {
    archive& boost::serialization::base_object<fuse_core::Constraint>(*this);
    archive& delta_;
    archive& sqrt_information_;
  }
};

}}  // namespace beam_constraints::frame_to_frame
 

BOOST_CLASS_EXPORT_KEY(
    beam_constraints::frame_to_frame::RelativeImuState3DStampedConstraint);
