#pragma once

#include <ostream>
#include <string>
#include <vector>

#include <fuse_core/constraint.h>
#include <fuse_core/eigen.h>
#include <fuse_core/macros.h>
#include <fuse_core/serialization.h>
#include <fuse_core/uuid.h>
#include <fuse_variables/point_3d_landmark.h>

#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <ceres/cost_function.h>
#include <ceres/autodiff_cost_function.h>


namespace beam_constraints {

class CameraLidarConstraint : public fuse_core::Constraint {
 public:
  FUSE_CONSTRAINT_DEFINITIONS(CameraLidarConstraint);

  /**
   * @brief Default constructor
   */
  CameraLidarConstraint() = default;

  /**
   * @brief Create a constraint using landmark location, camera pose and
   * measured pixel location
   *
   */
  CameraLidarConstraint(
      const std::string& source,
      const fuse_variables::Point3DLandmark& landmark,
      const Eigen::Vector3d& lidar_point);

  /**
   * @brief Destructor
   */
  virtual ~CameraLidarConstraint() = default;

  /**
   * @brief Read-only access to the lidar point
   *
   */
  const Eigen::Vector3d& LidarPoint() const { return lidar_point_; }

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
  Eigen::Vector3d lidar_point_;

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
    archive& lidar_point_;
  }
};

}  // namespace beam_constraints

BOOST_CLASS_EXPORT_KEY(beam_constraints::CameraLidarConstraint);