#include <beam_constraints/camera_to_camera/visual_constraint.h>
#include <beam_constraints/camera_to_camera/reprojection_functor.h>

#include <pluginlib/class_list_macros.h>

#include <boost/serialization/export.hpp>

#include <Eigen/Dense>

#include <string>
#include <vector>

namespace fuse_constraints {

VisualConstraint::VisualConstraint(
    const std::string& source,
    const fuse_variables::Orientation3DStamped& camera_orientation,
    const fuse_variables::Position3DStamped& camera_position,
    const fuse_variables::Position3D& landmark_position,
    const Eigen::Vector2d& pixel_measurement,
    const std::shared_ptr<beam_calibration::CameraModel> cam_model)
    : fuse_core::Constraint(source,
                            {camera_orientation.uuid(), camera_position.uuid(),
                             landmark_position.uuid()}) {
  pixel_ = pixel_measurement;
  cam_model_ = cam_model;
}

void VisualConstraint::print(std::ostream& stream) const {
  stream << type() << "\n"
         << "  source: " << source() << "\n"
         << "  uuid: " << uuid() << "\n"
         << "  pixel: " << pixel().transpose() << "\n";
}

ceres::CostFunction* VisualConstraint::costFunction() const {
  return new ceres::AutoDiffCostFunction<ReprojectionFunctor, 2, 4, 3, 3>(
      new ReprojectionFunctor(Eigen::Matrix2d::Identity(), pixel_, cam_model_));
}

} // namespace fuse_constraints
