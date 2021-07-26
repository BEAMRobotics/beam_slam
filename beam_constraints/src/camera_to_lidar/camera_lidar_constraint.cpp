#include <beam_constraints/camera_to_lidar/camera_lidar_constraint.h>

#include <string>
#include <vector>

#include <pluginlib/class_list_macros.h>
#include <boost/serialization/export.hpp>
#include <Eigen/Dense>

#include <beam_constraints/camera_to_lidar/3d_to_3d_cost_functor.h>

namespace beam_constraints {

CameraLidarConstraint::CameraLidarConstraint(
    const std::string& source, const fuse_variables::Position3D& landmark,
    const Eigen::Vector3d& lidar_point)
    : fuse_core::Constraint(source, {landmark.uuid()}),
      lidar_point_(lidar_point) {}

void CameraLidarConstraint::print(std::ostream& stream) const {
  stream << type() << "\n"
         << "  source: " << source() << "\n"
         << "  uuid: " << uuid() << "\n"
         << "  lidar point: " << LidarPoint().transpose() << "\n";
}

ceres::CostFunction* CameraLidarConstraint::costFunction() const {
  return new ceres::AutoDiffCostFunction<3dTo3DCostFunctor, 1, 3>(
      new 3dTo3DCostFunctor(lidar_point_));
}

}  // namespace beam_constraints
