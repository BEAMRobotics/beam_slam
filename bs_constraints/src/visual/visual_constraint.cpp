#include <bs_constraints/visual/reprojection_functor.h>
#include <bs_constraints/visual/unit_sphere_reprojection_functor.h>
#include <bs_constraints/visual/vilens_reprojection_functor.h>
#include <bs_constraints/visual/visual_constraint.h>
#include <fuse_loss/cauchy_loss.h>
#include <fuse_loss/huber_loss.h>
#include <fuse_loss/trivial_loss.h>

#include <pluginlib/class_list_macros.h>

#include <boost/serialization/export.hpp>

#include <Eigen/Dense>

#include <string>
#include <vector>

namespace fuse_constraints {

VisualConstraint::VisualConstraint(
    const std::string& source,
    const fuse_variables::Orientation3DStamped& R_WORLD_BASELINK,
    const fuse_variables::Position3DStamped& t_WORLD_BASELINK,
    const fuse_variables::Point3DLandmark& P_WORLD,
    const Eigen::Vector2d& pixel_measurement,
    const Eigen::Matrix4d& T_cam_baselink,
    const std::shared_ptr<beam_calibration::CameraModel> cam_model,
    const std::string& loss_type, const std::string& reproj_type)
    : fuse_core::Constraint(source, {R_WORLD_BASELINK.uuid(),
                                     t_WORLD_BASELINK.uuid(), P_WORLD.uuid()}) {
  pixel_ = pixel_measurement;
  cam_model_ = cam_model;
  T_cam_baselink_ = T_cam_baselink;

  if (reproj_type == "VANILLA" || reproj_type == "VILENS" ||
      reproj_type == "UNITSPHERE") {
    reprojection_type_ = reproj_type;
  } else {
    reprojection_type_ = "VANILLA";
    ROS_WARN_STREAM("Invalid reprojection type "
                    << reproj_type
                    << " using VANILLA. Options: VANILLA, VILENS, UNITSPHERE");
  }

  if (loss_type == "HUBER") {
    fuse_loss::HuberLoss::SharedPtr l =
        std::make_shared<fuse_loss::HuberLoss>();
    loss(l);
  } else if (loss_type == "CAUCHY") {
    fuse_loss::CauchyLoss::SharedPtr l =
        std::make_shared<fuse_loss::CauchyLoss>();
    loss(l);
  } else if (loss_type == "TRIVIAL") {
    fuse_loss::TrivialLoss::SharedPtr l =
        std::make_shared<fuse_loss::TrivialLoss>();
    loss(l);
  } else {
    ROS_WARN("Unsupported loss type '{}', using Trivial loss.", loss_type);
    fuse_loss::TrivialLoss::SharedPtr l =
        std::make_shared<fuse_loss::TrivialLoss>();
    loss(l);
  }
}

void VisualConstraint::print(std::ostream& stream) const {
  stream << type() << "\n"
         << "  source: " << source() << "\n"
         << "  uuid: " << uuid() << "\n"
         << "  pixel: " << pixel().transpose() << "\n";
}

ceres::CostFunction* VisualConstraint::costFunction() const {
  if (reprojection_type_ == "VANILLA") {
    return new ceres::AutoDiffCostFunction<ReprojectionFunctor, 2, 4, 3, 3>(
        new ReprojectionFunctor(pixel_, cam_model_, T_cam_baselink_));
  } else if (reprojection_type_ == "VILENS") {
    return new ceres::AutoDiffCostFunction<VilensReprojectionFunctor, 2, 4, 3,
                                           3>(
        new VilensReprojectionFunctor(pixel_, cam_model_, T_cam_baselink_));
  } else if (reprojection_type_ == "UNITSPHERE") {
    return new ceres::AutoDiffCostFunction<UnitSphereReprojectionFunctor, 2, 4,
                                           3, 3>(
        new UnitSphereReprojectionFunctor(pixel_, cam_model_, T_cam_baselink_));
  }
}

} // namespace fuse_constraints
