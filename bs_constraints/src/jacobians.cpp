#include <bs_constraints/jacobians.h>

namespace bs_constraints {

Eigen::Matrix<double, 2, 3>
    DImageProjectionDPoint(const Eigen::Matrix3d& camera_intrinsic_matrix,
                           const Eigen::Vector3d& P_CAMERA) {
  const auto fx = camera_intrinsic_matrix(0, 0);
  const auto fy = camera_intrinsic_matrix(1, 1);
  const auto x = P_CAMERA.x();
  const auto y = P_CAMERA.y();
  const auto z = P_CAMERA.z();
  const auto z2 = z * z;
  Eigen::Matrix<double, 2, 3> J;
  J << fx / z, 0, -fx * x / z2, 0, fy / z, -fy * y / z2;
  return J;
}

Eigen::Matrix<double, 3, 6>
    DPointTransformationDTransform(const Eigen::Matrix4d& T_frame_refframe,
                                   const Eigen::Vector3d& P_frame) {
  assert(beam::IsTransformationMatrix(T_frame_refframe));
  Eigen::Matrix<double, 3, 6> J = Eigen::Matrix<double, 3, 6>::Zero();
  const auto linear = T_frame_refframe.block<3, 3>(0, 0);
  const auto translation = T_frame_refframe.block<3, 1>(0, 3);
  J.block<3, 3>(0, 0) = linear;
  J.block<3, 3>(0, 3) = -2.0 * linear * beam::SkewTransform(P_frame);
  return J;
}

Eigen::Matrix3d
    DPointTransformationDPoint(const Eigen::Matrix4d& T_refframe_frame) {
  assert(beam::IsTransformationMatrix(T_refframe_frame));
  return T_refframe_frame.block<3, 3>(0, 0);
}

Eigen::Matrix<double, 6, 6>
    DInverseTransformDTransform(const Eigen::Matrix4d& T_refframe_frame) {
  assert(beam::IsTransformationMatrix(T_refframe_frame));
  Eigen::Matrix<double, 6, 6> J = Eigen::Matrix<double, 6, 6>::Zero();
  const auto linear = T_refframe_frame.block<3, 3>(0, 0);
  const auto translation = T_refframe_frame.block<3, 1>(0, 3);
  J.block<3, 3>(0, 0) = -linear;
  J.block<3, 3>(0, 3) =
      -2.0 * linear * beam::SkewTransform(linear.transpose() * translation);
  J.block<3, 3>(3, 3) = -linear;
  return J;
}

Eigen::Matrix<double, 6, 6>
    DTransformCompositionDRightTransform(const Eigen::Matrix4d& T_left,
                                         const Eigen::Matrix4d& T_right) {
  return Eigen::Matrix<double, 6, 6>::Identity();
}

Eigen::Matrix<double, 6, 6>
    DTransformCompositionDLeftTransform(const Eigen::Matrix4d& T_left,
                                        const Eigen::Matrix4d& T_right) {
  assert(beam::IsTransformationMatrix(T_right));
  Eigen::Matrix<double, 6, 6> J = Eigen::Matrix<double, 6, 6>::Zero();
  const auto linear = T_right.block<3, 3>(0, 0);
  const auto translation = T_right.block<3, 1>(0, 3);
  J.block<3, 3>(0, 0) = linear.transpose();
  J.block<3, 3>(0, 3) =
      -2.0 * linear.transpose() * beam::SkewTransform(translation);
  J.block<3, 3>(3, 3) = linear.transpose();
  return J;
}

} // namespace bs_constraints
