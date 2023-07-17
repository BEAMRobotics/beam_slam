#include <bs_constraints/visual/euclidean_reprojection_constraint.h>
#include <bs_constraints/visual/euclidean_reprojection_function.h>
#include <bs_constraints/visual/reprojection_functor.h>

#include <pluginlib/class_list_macros.h>

#include <boost/serialization/export.hpp>

#include <Eigen/Dense>

#include <string>
#include <vector>

namespace bs_constraints {

EuclideanReprojectionConstraint::EuclideanReprojectionConstraint(
    const std::string& source,
    const fuse_variables::Orientation3DStamped& R_WORLD_BASELINK,
    const fuse_variables::Position3DStamped& t_WORLD_BASELINK,
    const fuse_variables::Point3DLandmark& P_WORLD,
    const Eigen::Matrix4d& T_cam_baselink,
    const Eigen::Matrix3d& intrinsic_matrix, const Eigen::Vector2d& measurement,
    const double reprojection_information_weight)
    : fuse_core::Constraint(source, {R_WORLD_BASELINK.uuid(),
                                     t_WORLD_BASELINK.uuid(), P_WORLD.uuid()}),
      T_cam_baselink_(T_cam_baselink),
      intrinsic_matrix_(intrinsic_matrix),
      pixel_(measurement),
      sqrt_information_(reprojection_information_weight *
                        Eigen::Matrix2d::Identity()) {}

void EuclideanReprojectionConstraint::print(std::ostream& stream) const {
  stream << type() << "\n"
         << "  source: " << source() << "\n"
         << "  uuid: " << uuid() << "\n"
         << "  pixel: " << pixel().transpose() << "\n";
}

ceres::CostFunction* EuclideanReprojectionConstraint::costFunction() const {
  return new EuclideanReprojection(sqrt_information_, pixel_, intrinsic_matrix_,
                                   T_cam_baselink_);
  // return new ceres::AutoDiffCostFunction<ReprojectionFunctor, 2, 4, 3, 3>(
  //     new ReprojectionFunctor(sqrt_information_, pixel_, intrinsic_matrix_,
  //                             T_cam_baselink_));
}

} // namespace bs_constraints

BOOST_CLASS_EXPORT_IMPLEMENT(bs_constraints::EuclideanReprojectionConstraint);
PLUGINLIB_EXPORT_CLASS(bs_constraints::EuclideanReprojectionConstraint,
                       fuse_core::Constraint);
