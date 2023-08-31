#include <bs_constraints/visual/inversedepth_reprojection_constraint.h>
#include <bs_constraints/visual/inversedepth_reprojection_functor.h>

#include <pluginlib/class_list_macros.h>

#include <boost/serialization/export.hpp>

#include <Eigen/Dense>

#include <string>
#include <vector>

namespace bs_constraints {

InverseDepthReprojectionConstraint::InverseDepthReprojectionConstraint(
    const std::string& source,
    const fuse_variables::Orientation3DStamped& R_WORLD_BASELINKa,
    const fuse_variables::Position3DStamped& t_WORLD_BASELINKa,
    const fuse_variables::Orientation3DStamped& R_WORLD_BASELINKm,
    const fuse_variables::Position3DStamped& t_WORLD_BASELINKm,
    const bs_variables::InverseDepthLandmark& idp,
    const Eigen::Matrix4d& T_cam_baselink,
    const Eigen::Matrix3d& intrinsic_matrix, const Eigen::Vector2d& measurement,
    const double reprojection_information_weight)
    : fuse_core::Constraint(source,
                            {R_WORLD_BASELINKa.uuid(), t_WORLD_BASELINKa.uuid(),
                             R_WORLD_BASELINKm.uuid(), t_WORLD_BASELINKm.uuid(),
                             idp.uuid()}),
      T_cam_baselink_(T_cam_baselink),
      intrinsic_matrix_(intrinsic_matrix),
      pixel_(measurement),
      bearing_(idp.bearing()),
      sqrt_information_(reprojection_information_weight *
                        Eigen::Matrix2d::Identity()) {}

void InverseDepthReprojectionConstraint::print(std::ostream& stream) const {
  stream << type() << "\n"
         << "  source: " << source() << "\n"
         << "  uuid: " << uuid() << "\n"
         << "  pixel: " << pixel().transpose() << "\n";
}

ceres::CostFunction* InverseDepthReprojectionConstraint::costFunction() const {
  return new ceres::AutoDiffCostFunction<InverseDepthReprojectionFunctor, 2, 4,
                                         3, 4, 3, 1>(
      new InverseDepthReprojectionFunctor(sqrt_information_, pixel_,
                                          intrinsic_matrix_, T_cam_baselink_,
                                          bearing_));
}

} // namespace bs_constraints

BOOST_CLASS_EXPORT_IMPLEMENT(
    bs_constraints::InverseDepthReprojectionConstraint);
PLUGINLIB_EXPORT_CLASS(bs_constraints::InverseDepthReprojectionConstraint,
                       fuse_core::Constraint);
