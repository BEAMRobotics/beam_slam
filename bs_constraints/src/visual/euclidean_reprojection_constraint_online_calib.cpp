#include <bs_constraints/visual/euclidean_reprojection_constraint_online_calib.h>
#include <bs_constraints/visual/euclidean_reprojection_functor_online_calib.h>

#include <pluginlib/class_list_macros.h>

#include <boost/serialization/export.hpp>

#include <Eigen/Dense>

#include <string>
#include <vector>

namespace bs_constraints {

EuclideanReprojectionConstraintOnlineCalib::
    EuclideanReprojectionConstraintOnlineCalib(
        const std::string& source,
        const fuse_variables::Orientation3DStamped& R_WORLD_BASELINK,
        const fuse_variables::Position3DStamped& t_WORLD_BASELINK,
        const bs_variables::Point3DLandmark& P_WORLD,
        const bs_variables::Orientation3D& R_CAM_BASELINK,
        const bs_variables::Position3D& t_CAM_BASELINK,
        const Eigen::Matrix3d& intrinsic_matrix,
        const Eigen::Vector2d& measurement,
        const double reprojection_information_weight)
    : fuse_core::Constraint(source,
                            {R_WORLD_BASELINK.uuid(), t_WORLD_BASELINK.uuid(),
                             P_WORLD.uuid(), R_CAM_BASELINK.uuid(),
                             t_CAM_BASELINK.uuid()}),
      intrinsic_matrix_(intrinsic_matrix),
      pixel_(measurement),
      sqrt_information_(reprojection_information_weight *
                        Eigen::Matrix2d::Identity()) {}

void EuclideanReprojectionConstraintOnlineCalib::print(
    std::ostream& stream) const {
  stream << type() << "\n"
         << "  source: " << source() << "\n"
         << "  uuid: " << uuid() << "\n"
         << "  pixel: " << pixel().transpose() << "\n";
}

ceres::CostFunction*
    EuclideanReprojectionConstraintOnlineCalib::costFunction() const {
  return new ceres::AutoDiffCostFunction<EuclideanReprojectionFunctorOnlineCalib, 2, 4, 3,
                                         3, 4, 3>(
      new EuclideanReprojectionFunctorOnlineCalib(sqrt_information_, pixel_,
                                                  intrinsic_matrix_));
}

} // namespace bs_constraints

BOOST_CLASS_EXPORT_IMPLEMENT(
    bs_constraints::EuclideanReprojectionConstraintOnlineCalib);
PLUGINLIB_EXPORT_CLASS(
    bs_constraints::EuclideanReprojectionConstraintOnlineCalib,
    fuse_core::Constraint);
