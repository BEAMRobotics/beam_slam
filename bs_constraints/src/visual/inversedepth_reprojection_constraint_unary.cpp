#include <bs_constraints/visual/inversedepth_reprojection_constraint_unary.h>
#include <bs_constraints/visual/inversedepth_reprojection_functor_unary.h>

#include <pluginlib/class_list_macros.h>

#include <boost/serialization/export.hpp>

#include <Eigen/Dense>

#include <string>
#include <vector>

namespace bs_constraints {

InverseDepthReprojectionConstraintUnary::
    InverseDepthReprojectionConstraintUnary(
        const std::string& source,
        const fuse_variables::Orientation3DStamped& o_WORLD_BASELINKa,
        const fuse_variables::Position3DStamped& p_WORLD_BASELINKa,
        const bs_variables::InverseDepthLandmark& idp,
        const Eigen::Matrix4d& T_cam_baselink,
        const Eigen::Matrix3d& intrinsic_matrix,
        const Eigen::Vector2d& measurement,
        const double reprojection_information_weight)
    : fuse_core::Constraint(source, {o_WORLD_BASELINKa.uuid(),
                                     p_WORLD_BASELINKa.uuid(), idp.uuid()}),
      T_cam_baselink_(T_cam_baselink),
      intrinsic_matrix_(intrinsic_matrix),
      pixel_(measurement),
      bearing_(idp.bearing()),
      sqrt_information_(reprojection_information_weight *
                        Eigen::Matrix2d::Identity()) {}

void InverseDepthReprojectionConstraintUnary::print(
    std::ostream& stream) const {
  stream << type() << "\n"
         << "  source: " << source() << "\n"
         << "  uuid: " << uuid() << "\n"
         << "  pixel: " << pixel().transpose() << "\n";
}

ceres::CostFunction*
    InverseDepthReprojectionConstraintUnary::costFunction() const {
  return new ceres::AutoDiffCostFunction<InverseDepthReprojectionFunctorUnary,
                                         2, 4, 3, 1>(
      new InverseDepthReprojectionFunctorUnary(sqrt_information_, pixel_,
                                               intrinsic_matrix_,
                                               T_cam_baselink_, bearing_));
}

} // namespace bs_constraints

BOOST_CLASS_EXPORT_IMPLEMENT(
    bs_constraints::InverseDepthReprojectionConstraintUnary);
PLUGINLIB_EXPORT_CLASS(bs_constraints::InverseDepthReprojectionConstraintUnary,
                       fuse_core::Constraint);
