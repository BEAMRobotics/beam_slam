#include <bs_constraints/jacobians.h>

inline void QuaternionInverse(const double in[4], double out[4]) {
  out[0] = in[0];
  out[1] = -in[1];
  out[2] = -in[2];
  out[3] = -in[3];
}

inline Eigen::Matrix4d Oplus(const Eigen::Quaterniond& q_BC) {
  Eigen::Vector4d q = q_BC.coeffs();
  Eigen::Matrix4d Q;
  // clang-format off
  Q(0,0) =  q[3]; Q(0,1) =  q[2]; Q(0,2) = -q[1]; Q(0,3) =  q[0];
  Q(1,0) = -q[2]; Q(1,1) =  q[3]; Q(1,2) =  q[0]; Q(1,3) =  q[1];
  Q(2,0) =  q[1]; Q(2,1) = -q[0]; Q(2,2) =  q[3]; Q(2,3) =  q[2];
  Q(3,0) = -q[0]; Q(3,1) = -q[1]; Q(3,2) = -q[2]; Q(3,3) =  q[3];
  // clang-format on
  return Q;
}

namespace bs_constraints {

Eigen::Quaterniond SO3BoxPlus(const Eigen::Quaterniond& q,
                              const Eigen::Vector3d& pert) {
  double x[4] = {q.w(), q.x(), q.y(), q.z()};
  double delta[3] = {pert[0], pert[1], pert[2]};
  double q_delta[4];
  double x_plus_delta[4];
  ceres::AngleAxisToQuaternion(delta, q_delta);
  ceres::QuaternionProduct(x, q_delta, x_plus_delta);
  Eigen::Quaterniond q_pert(x_plus_delta[0], x_plus_delta[1], x_plus_delta[2],
                            x_plus_delta[3]);
  return q_pert;
}

Eigen::Vector3d SO3BoxMinus(const Eigen::Quaterniond& q1,
                            const Eigen::Quaterniond& q2) {
  double x1[4] = {q1.w(), q1.x(), q1.y(), q1.z()};
  double x2[4] = {q2.w(), q2.x(), q2.y(), q2.z()};
  double delta[3];

  double x1_inverse[4];
  QuaternionInverse(x1, x1_inverse);
  double q_delta[4];
  ceres::QuaternionProduct(x1_inverse, x2, q_delta);
  ceres::QuaternionToAngleAxis(q_delta, delta);
  Eigen::Vector3d delta_out(delta[0], delta[1], delta[2]);
  return delta_out;
}

Eigen::Matrix<double, 7, 1>
    TranslationSO3BoxPlus(const Eigen::Matrix<double, 7, 1>& T,
                          const Eigen::Matrix<double, 6, 1>& pert) {
  Eigen::Quaterniond q(T[3], T[4], T[5], T[6]);
  Eigen::Vector3d t(T[0], T[1], T[2]);
  Eigen::Vector3d q_delta = pert.tail<3>();
  Eigen::Vector3d t_delta = pert.head<3>();
  const auto q_plus_delta = SO3BoxPlus(q, q_delta);
  const auto t_plus_delta = t + t_delta;
  Eigen::Matrix<double, 7, 1> T_plus_delta;
  T_plus_delta << t_plus_delta.x(), t_plus_delta.y(), t_plus_delta.z(),
      q_plus_delta.w(), q_plus_delta.x(), q_plus_delta.y(), q_plus_delta.z();
  return T_plus_delta;
}

Eigen::Matrix<double, 6, 1>
    TranslationSO3BoxMinus(const Eigen::Matrix<double, 7, 1>& T1,
                           const Eigen::Matrix<double, 7, 1>& T2) {
  Eigen::Quaterniond q1(T1[3], T1[4], T1[5], T1[6]);
  Eigen::Quaterniond q2(T2[3], T2[4], T2[5], T2[6]);
  const auto q_delta = SO3BoxMinus(q1, q2);
  Eigen::Vector3d t1(T1[0], T1[1], T1[2]);
  Eigen::Vector3d t2(T2[0], T2[1], T2[2]);
  const auto t_delta = t1 - t2;
  Eigen::Matrix<double, 6, 1> delta;
  delta << t_delta[0], t_delta[1], t_delta[2], q_delta[0], q_delta[1],
      q_delta[2];
  return delta;
}

Eigen::Matrix<double, 7, 1>
    SE3BoxPlus(const Eigen::Matrix<double, 7, 1>& T,
               const Eigen::Matrix<double, 6, 1>& pert) {
  Eigen::Quaterniond q(T[3], T[4], T[5], T[6]);
  Eigen::Vector3d t(T[0], T[1], T[2]);
  Eigen::Matrix4d T_mat = Eigen::Matrix4d::Identity();
  T_mat.block<3, 3>(0, 0) = q.toRotationMatrix();
  T_mat.block<3, 1>(0, 3) = t;

  Eigen::Vector3d R_delta = pert.tail<3>();
  double q_delta_d[4];
  ceres::AngleAxisToQuaternion(R_delta.data(), q_delta_d);
  Eigen::Quaterniond q_delta(q_delta_d[0], q_delta_d[1], q_delta_d[2],
                             q_delta_d[3]);
  Eigen::Vector3d t_delta = pert.head<3>();
  Eigen::Matrix4d T_delta_mat = Eigen::Matrix4d::Identity();
  T_delta_mat.block<3, 3>(0, 0) = q_delta.toRotationMatrix();
  T_delta_mat.block<3, 1>(0, 3) = t_delta;

  auto T_plus_delta_mat = T_mat * T_delta_mat;
  Eigen::Vector3d t_plus_delta = T_plus_delta_mat.block<3, 1>(0, 3);
  Eigen::Matrix3d R_plus_delta = T_plus_delta_mat.block<3, 3>(0, 0);
  Eigen::Quaterniond q_plus_delta(R_plus_delta);

  Eigen::Matrix<double, 7, 1> T_plus_delta;
  T_plus_delta << t_plus_delta.x(), t_plus_delta.y(), t_plus_delta.z(),
      q_plus_delta.w(), q_plus_delta.x(), q_plus_delta.y(), q_plus_delta.z();
  return T_plus_delta;
}

Eigen::Matrix<double, 6, 1> SE3BoxMinus(const Eigen::Matrix<double, 7, 1>& T1,
                                        const Eigen::Matrix<double, 7, 1>& T2) {
  Eigen::Quaterniond q1(T1[3], T1[4], T1[5], T1[6]);
  Eigen::Quaterniond q2(T2[3], T2[4], T2[5], T2[6]);
  Eigen::Vector3d t1(T1[0], T1[1], T1[2]);
  Eigen::Vector3d t2(T2[0], T2[1], T2[2]);

  Eigen::Matrix4d T1_mat = Eigen::Matrix4d::Identity();
  T1_mat.block<3, 3>(0, 0) = q1.toRotationMatrix();
  T1_mat.block<3, 1>(0, 3) = t1;

  Eigen::Matrix4d T2_mat = Eigen::Matrix4d::Identity();
  T2_mat.block<3, 3>(0, 0) = q2.toRotationMatrix();
  T2_mat.block<3, 1>(0, 3) = t2;

  const auto delta_mat = beam::InvertTransform(T2_mat) * T1_mat;
  Eigen::Quaterniond q_delta(delta_mat.block<3, 3>(0, 0));
  Eigen::Vector3d t_delta(delta_mat.block<3, 1>(0, 3));

  double R_delta[3];
  double q_delta_data[4] = {q_delta.w(), q_delta.x(), q_delta.y(), q_delta.z()};
  ceres::QuaternionToAngleAxis(q_delta_data, R_delta);

  Eigen::Matrix<double, 6, 1> delta;
  delta << t_delta[0], t_delta[1], t_delta[2], R_delta[0], R_delta[1],
      R_delta[2];
  return delta;
}

Eigen::Matrix<double, 4, 3> PlusJacobian(const Eigen::Quaterniond& q) {
  double x0 = q.w() / 2;
  double x1 = q.x() / 2;
  double x2 = q.y() / 2;
  double x3 = q.z() / 2;
  Eigen::Matrix<double, 4, 3> jacobian;
  // clang-format off
  jacobian(0,0) = -x1; jacobian(0,1) = -x2; jacobian(0,2) = -x3; 
  jacobian(1,0) =  x0; jacobian(1,1) = -x3; jacobian(1,2) =  x2; 
  jacobian(2,0) =  x3; jacobian(2,1) =  x0; jacobian(2,2) = -x1; 
  jacobian(3,0) = -x2; jacobian(3,1) =  x1; jacobian(3,2) =  x0;
  // clang-format on
  return jacobian;
}

Eigen::Matrix<double, 3, 4> MinusJacobian(const Eigen::Quaterniond& q) {
  double x0 = q.w() * 2;
  double x1 = q.x() * 2;
  double x2 = q.y() * 2;
  double x3 = q.z() * 2;
  Eigen::Matrix<double, 3, 4> jacobian;
  // clang-format off
  jacobian(0,0) = -x1; jacobian(0,1) =  x0; jacobian(0,2) =  x3;  jacobian(0,3) = -x2; 
  jacobian(1,0) = -x2; jacobian(1,1) = -x3; jacobian(1,2) =  x0;  jacobian(1,3) =  x1;  
  jacobian(2,0) = -x3; jacobian(2,1) =  x2; jacobian(2,2) = -x1;  jacobian(2,3) =  x0;
  // clang-format on
  return jacobian;
}

Eigen::Matrix3d DPointRotationDRotation(const Eigen::Matrix3d& R,
                                        const Eigen::Vector3d& point) {
  return -R * beam::SkewX(point);
}

Eigen::Matrix3d DPointRotationDPoint(const Eigen::Matrix3d& R,
                                     const Eigen::Vector3d& point) {
  return R;
}

Eigen::Matrix3d DInverseRotationDRotation(const Eigen::Matrix3d& R) {
  return -R;
}

Eigen::Matrix3d
    DRotationCompositionDLeftRotation(const Eigen::Matrix3d& R_left,
                                      const Eigen::Matrix3d& R_right) {
  return R_right.transpose();
}

Eigen::Matrix3d
    DRotationCompositionDRightRotation(const Eigen::Matrix3d& R_left,
                                       const Eigen::Matrix3d& R_right) {
  return Eigen::Matrix3d::Identity();
}

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
  J.block<3, 3>(0, 3) = DPointRotationDRotation(linear, P_frame);
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
      DPointRotationDRotation(linear, linear.transpose() * translation);
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
      DPointRotationDRotation(linear.transpose(), translation);
  J.block<3, 3>(3, 3) = linear.transpose();
  return J;
}

} // namespace bs_constraints
