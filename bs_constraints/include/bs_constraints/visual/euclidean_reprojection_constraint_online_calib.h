#pragma once

#include <bs_variables/point_3d_landmark.h>
#include <fuse_core/constraint.h>
#include <fuse_core/eigen.h>
#include <fuse_core/fuse_macros.h>
#include <fuse_core/serialization.h>
#include <fuse_core/uuid.h>
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>
#include <bs_variables/orientation_3d.h>
#include <bs_variables/position_3d.h>

#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <ceres/autodiff_cost_function.h>
#include <ceres/cost_function.h>

#include <ostream>
#include <string>
#include <vector>

namespace bs_constraints {

class EuclideanReprojectionConstraintOnlineCalib
    : public fuse_core::Constraint {
public:
  FUSE_CONSTRAINT_DEFINITIONS(EuclideanReprojectionConstraintOnlineCalib);

  /**
   * @brief Default constructor
   */
  EuclideanReprojectionConstraintOnlineCalib() = default;

  /**
   * @brief Create a constraint using landmark location, camera pose and
   * measured pixel location
   *
   */
  EuclideanReprojectionConstraintOnlineCalib(
      const std::string& source,
      const fuse_variables::Orientation3DStamped& R_WORLD_BASELINK,
      const fuse_variables::Position3DStamped& t_WORLD_BASELINK,
      const bs_variables::Point3DLandmark& P_WORLD,
      const bs_variables::Orientation3D& R_CAM_BASELINK,
      const bs_variables::Position3D& t_CAM_BASELINK,
      const Eigen::Matrix3d& intrinsic_matrix,
      const Eigen::Vector2d& measurement,
      const double reprojection_information_weight);

  /**
   * @brief Destructor
   */
  virtual ~EuclideanReprojectionConstraintOnlineCalib() = default;

  /**
   * @brief Read-only access to the measured pixel value
   *
   */
  const Eigen::Vector2d& pixel() const { return pixel_; }

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
  Eigen::Vector2d pixel_;
  Eigen::Matrix3d intrinsic_matrix_;
  Eigen::Matrix2d sqrt_information_;

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
    archive& pixel_;
  }
};

} // namespace bs_constraints

BOOST_CLASS_EXPORT_KEY(
    bs_constraints::EuclideanReprojectionConstraintOnlineCalib);