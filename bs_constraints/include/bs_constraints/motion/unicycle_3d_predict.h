#pragma once

#include <fuse_core/util.h>

#include <tf2/LinearMath/Transform.h>

namespace fuse_core {
using Matrix15d = Eigen::Matrix<double, 15, 15, Eigen::RowMajor>;
using Vector15d = Eigen::Matrix<double, 15, 1>;
} // namespace fuse_core

namespace bs_constraints { namespace motion {

/**
 * @brief Given a state and time delta, predicts a new state
 * @param[in] position1_x - First X position
 * @param[in] position1_y - First Y position
 * @param[in] position1_z - First Z position
 * @param[in] roll1  - First roll orientation
 * @param[in] pitch1 - First pitch orientation
 * @param[in] yaw1   - First yaw orientation
 * @param[in] vel_linear1_x - First X velocity
 * @param[in] vel_linear1_y - First Y velocity
 * @param[in] vel_linear1_z - First Z velocity
 * @param[in] vel_roll1  - First roll velocity
 * @param[in] vel_pitch1 - First pitch velocity
 * @param[in] vel_yaw1   - First yaw velocity
 * @param[in] acc_linear1_x - First X acceleration
 * @param[in] acc_linear1_y - First Y acceleration
 * @param[in] acc_linear1_z - First Z acceleration
 * @param[in] dt - The time delta across which to predict the state
 * @param[in] position2_x - Second X position
 * @param[in] position2_y - Second Y position
 * @param[in] position2_z - Second Z position
 * @param[in] roll2  - Second roll orientation
 * @param[in] pitch2 - Second pitch orientation
 * @param[in] yaw2   - Second yaw orientation
 * @param[in] vel_linear2_x - Second X velocity
 * @param[in] vel_linear2_y - Second Y velocity
 * @param[in] vel_linear2_z - Second Z velocity
 * @param[in] vel_roll2  - Second roll velocity
 * @param[in] vel_pitch2 - Second pitch velocity
 * @param[in] vel_yaw2   - Second yaw velocity
 * @param[in] acc_linear2_x - Second X acceleration
 * @param[in] acc_linear2_y - Second Y acceleration
 * @param[in] acc_linear2_z - Second Z acceleration
 */
template <typename T>
inline void predict(const T position1_x, const T position1_y,
                    const T position1_z, const T roll1, const T pitch1,
                    const T yaw1, const T vel_linear1_x, const T vel_linear1_y,
                    const T vel_linear1_z, const T vel_roll1,
                    const T vel_pitch1, const T vel_yaw1, const T acc_linear1_x,
                    const T acc_linear1_y, const T acc_linear1_z, const T dt,
                    T& position2_x, T& position2_y, T& position2_z, T& roll2,
                    T& pitch2, T& yaw2, T& vel_linear2_x, T& vel_linear2_y,
                    T& vel_linear2_z, T& vel_roll2, T& vel_pitch2, T& vel_yaw2,
                    T& acc_linear2_x, T& acc_linear2_y, T& acc_linear2_z) {
  T sp = ceres::sin(pitch1);
  T cp = ceres::cos(pitch1);
  T cpi = 1.0 / cp;
  T tp = sp * cpi;

  T sr = ceres::sin(roll1);
  T cr = ceres::cos(roll1);

  T sy = ceres::sin(yaw1);
  T cy = ceres::cos(yaw1);

  T VX = vel_linear1_x;
  T VY = vel_linear1_y;
  T VZ = vel_linear1_z;

  T AX = acc_linear1_x;
  T AY = acc_linear1_y;
  T AZ = acc_linear1_z;
  T delta = dt;

  T VR = vel_roll1;
  T VP = vel_pitch1;
  T VYAW = vel_yaw1;

  // Prepare the transfer function

  T X_VX = cy * cp * delta;
  T X_VY = (cy * sp * sr - sy * cr) * delta;
  T X_VZ = (cy * sp * cr + sy * sr) * delta;
  T X_AX = 0.5 * X_VX * delta;
  T X_AY = 0.5 * X_VY * delta;
  T X_AZ = 0.5 * X_VZ * delta;

  T Y_VX = sy * cp * delta;
  T Y_VY = (sy * sp * sr + cy * cr) * delta;
  T Y_VZ = (sy * sp * cr - cy * sr) * delta;
  T Y_AX = 0.5 * Y_VX * delta;
  T Y_AY = 0.5 * Y_VY * delta;
  T Y_AZ = 0.5 * Y_VZ * delta;

  T Z_VX = -sp * delta;
  T Z_VY = cp * sr * delta;
  T Z_VZ = (cp * cr * delta);
  T Z_AX = (0.5 * Z_VX) * delta;
  T Z_AY = (0.5 * Z_VY) * delta;
  T Z_AZ = (0.5 * Z_VZ) * delta;

  T ROLL_VR = delta;
  T ROLL_VP = (sr * tp * delta);
  T ROLL_VY = (cr * tp * delta);

  T PITCH_VP = (cr * delta);
  T PITCH_VY = (-sr * delta);

  T YAW_VP = (sr * cpi * delta);
  T YAW_VY = (cr * cpi * delta);

  T VX_AX = delta;
  T VY_AY = delta;
  T VZ_AZ = delta;

  position2_x = position1_x + VX * X_VX + VY * X_VY + VZ * X_VZ + AX * X_AX +
                AY * X_AY + AZ * X_AZ;
  position2_y = position1_y + VX * Y_VX + VY * Y_VY + VZ * Y_VZ + AX * Y_AX +
                AY * Y_AY + AZ * Y_AZ;
  position2_z = position1_z + VX * Z_VX + VY * Z_VY + VZ * Z_VZ + AX * Z_AX +
                AY * Z_AY + AZ * Z_AZ;

  roll2 = roll1 + VR * ROLL_VR + VP * ROLL_VP + VYAW * ROLL_VY;
  pitch2 = pitch1 + VP * PITCH_VP + VYAW * PITCH_VY;
  yaw2 = yaw1 + VP * YAW_VP + VYAW * YAW_VY;

  vel_linear2_x = vel_linear1_x + AX * VX_AX;
  vel_linear2_y = vel_linear1_y + AY * VY_AY;
  vel_linear2_z = vel_linear1_z + AZ * VZ_AZ;

  vel_roll2 = vel_roll1;
  vel_pitch2 = vel_pitch1;
  vel_yaw2 = vel_yaw1;

  acc_linear2_x = acc_linear1_x;
  acc_linear2_y = acc_linear1_y;
  acc_linear2_z = acc_linear1_z;

  fuse_core::wrapAngle2D(roll2);
  fuse_core::wrapAngle2D(pitch2);
  fuse_core::wrapAngle2D(yaw2);
}

/**
 * @brief Given a state and time delta, predicts a new state
 * @param[in] position1 - First position (array with x at index 0, y at index 1)
 * @param[in] yaw1 - First orientation
 * @param[in] vel_linear1 - First velocity (array with x at index 0, y at index
 * 1)
 * @param[in] vel_yaw1 - First yaw velocity
 * @param[in] acc_linear1 - First linear acceleration (array with x at index 0,
 * y at index 1)
 * @param[in] dt - The time delta across which to predict the state
 * @param[out] position2 - Second position (array with x at index 0, y at index
 * 1)
 * @param[out] yaw2 - Second orientation
 * @param[out] vel_linear2 - Second velocity (array with x at index 0, y at
 * index 1)
 * @param[out] vel_yaw2 - Second yaw velocity
 * @param[out] acc_linear2 - Second linear acceleration (array with x at index
 * 0, y at index 1)
 */
template <typename T>
inline void predict(const T* const position1, const T* const orientation1,
                    const T* const vel_linear1, const T* const vel_angular1,
                    const T* const acc_linear1, const T dt, T* const position2,
                    T* const orientation2, T* const vel_linear2,
                    T* const vel_angular2, T* const acc_linear2) {
  T roll1 = fuse_core::getRoll(orientation1[0], orientation1[1],
                               orientation1[2], orientation1[3]);
  T pitch1 = fuse_core::getPitch(orientation1[0], orientation1[1],
                                 orientation1[2], orientation1[3]);
  T yaw1 = fuse_core::getYaw(orientation1[0], orientation1[1], orientation1[2],
                             orientation1[3]);

  predict(position1[0], position1[1], position1[2], roll1, pitch1, yaw1,
          vel_linear1[0], vel_linear1[1], vel_linear1[2], vel_angular1[0],
          vel_angular1[1], vel_angular1[2], acc_linear1[0], acc_linear1[1],
          acc_linear1[2], dt, position2[0], position2[1], position2[2],
          orientation2[0], orientation2[1], orientation2[2], vel_linear2[0],
          vel_linear2[1], vel_linear2[2], vel_angular2[0], vel_angular2[1],
          vel_angular2[2], acc_linear2[0], acc_linear2[1], acc_linear2[2]);
}

/**
 * @brief Given a state and time delta, predicts a new state
 * @param[in] pose1 - The first 2D pose
 * @param[in] vel_linear_1 - The first linear velocity
 * @param[in] vel_yaw1 - The first yaw velocity
 * @param[in] acc_linear1 - The first linear acceleration
 * @param[in] dt - The time delta across which to predict the state
 * @param[in] pose2 - The second 2D pose
 * @param[in] vel_linear_2 - The second linear velocity
 * @param[in] vel_yaw2 - The second yaw velocity
 * @param[in] acc_linear2 - The second linear acceleration
 */

inline void predict(const tf2::Transform& pose1,
                    const tf2::Vector3& vel_linear1,
                    const tf2::Vector3& vel_angular1,
                    const tf2::Vector3& acc_linear1, const double dt,
                    tf2::Transform& pose2, tf2::Vector3& vel_linear2,
                    tf2::Vector3& vel_angular2, tf2::Vector3& acc_linear2) {
  double x_pred{};
  double y_pred{};
  double z_pred{};

  double roll_pred{};
  double pitch_pred{};
  double yaw_pred{};

  double vel_linear_x_pred{};
  double vel_linear_y_pred{};
  double vel_linear_z_pred{};

  double vel_angular_x_pred{};
  double vel_angular_y_pred{};
  double vel_angular_z_pred{};

  double acc_linear_x_pred{};
  double acc_linear_y_pred{};
  double acc_linear_z_pred{};

  double roll1, pitch1, yaw1;
  pose1.getBasis().getRPY(roll1, pitch1, yaw1);

  predict(pose1.getOrigin().x(), pose1.getOrigin().y(), pose1.getOrigin().z(),
          roll1, pitch1, yaw1, vel_linear1.x(), vel_linear1.y(),
          vel_linear1.z(), vel_angular1.x(), vel_angular1.y(), vel_angular1.z(),
          acc_linear1.x(), acc_linear1.y(), acc_linear1.z(), dt, x_pred, y_pred,
          z_pred, roll_pred, pitch_pred, yaw_pred, vel_linear_x_pred,
          vel_linear_y_pred, vel_linear_z_pred, vel_angular_x_pred,
          vel_angular_y_pred, vel_angular_z_pred, acc_linear_x_pred,
          acc_linear_y_pred, acc_linear_z_pred);

  pose2.setOrigin(tf2::Vector3{x_pred, y_pred, z_pred});
  tf2::Quaternion q;
  q.setRPY(roll_pred, pitch_pred, yaw_pred);
  pose2.setRotation(q);

  vel_linear2.setX(vel_linear_x_pred);
  vel_linear2.setY(vel_linear_y_pred);
  vel_linear2.setZ(vel_linear_z_pred);

  vel_angular2.setX(vel_angular_x_pred);
  vel_angular2.setY(vel_angular_y_pred);
  vel_angular2.setZ(vel_angular_z_pred);

  acc_linear2.setX(acc_linear_x_pred);
  acc_linear2.setY(acc_linear_y_pred);
  acc_linear2.setZ(acc_linear_z_pred);
};

}} // namespace bs_constraints::motion
