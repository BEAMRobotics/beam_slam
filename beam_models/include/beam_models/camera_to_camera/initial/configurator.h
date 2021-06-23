#pragma once

#include <beam_calibration/CameraModels.h>
#include <slamtools/configurator.h>

namespace beam_models { namespace camera_to_camera {

class CameraConfig : public Configurator {
public:
  CameraConfig() = default;

  CameraConfig(std::shared_ptr<beam_calibration::CameraModel> cam_model,
               const Eigen::Matrix4d& T_body_cam,
               const Eigen::Matrix4d& T_body_imu,
               const Eigen::Vector4d& imu_intrinsics) {
    // set camera intrinsics
    Eigen::VectorXd intrinsics = cam_model->GetIntrinsics();
    K.setIdentity();
    K(0, 0) = intrinsics[0];
    K(1, 1) = intrinsics[1];
    K(0, 2) = intrinsics[2];
    K(1, 2) = intrinsics[3];
    // set imu intrinsics
    const double sigma_correction = 1.0;
    double sigma_w = imu_intrinsics[0] * sigma_correction;  // gryo noise
    double sigma_a = imu_intrinsics[1] * sigma_correction;  // accel noise
    double sigma_bg = imu_intrinsics[2] * sigma_correction; // gryo random
    double sigma_ba = imu_intrinsics[3] * sigma_correction; // accel random
    gyro_noise = sigma_w * sigma_w * Eigen::Matrix3d::Identity();
    accl_noise = sigma_a * sigma_a * Eigen::Matrix3d::Identity();
    gyro_random_walk = sigma_bg * sigma_bg * Eigen::Matrix3d::Identity();
    accl_random_walk = sigma_ba * sigma_ba * Eigen::Matrix3d::Identity();
    // set cam to body transform
    Eigen::Matrix3d R_body_cam = T_body_cam.block<3, 3>(0, 0);
    Eigen::Quaterniond q_body_cam(R_body_cam);
    q_cam2body = q_body_cam;
    Eigen::Vector3d t_body_cam = T_body_cam.block<3, 1>(0, 3).transpose();
    p_cam2body = t_body_cam;
    // set imu to body transform
    Eigen::Matrix3d R_body_imu = T_body_imu.block<3, 3>(0, 0);
    Eigen::Quaterniond q_body_imu(R_body_imu);
    q_imu2body = q_body_imu;
    Eigen::Vector3d t_body_imu = T_body_imu.block<3, 1>(0, 3).transpose();
    p_imu2body = t_body_imu;
  }

  ~CameraConfig() = default;

  Eigen::Matrix3d camera_intrinsic() const { return K; }

  Eigen::Quaterniond camera_to_center_rotation() const { return q_cam2body; }

  Eigen::Vector3d camera_to_center_translation() const { return p_cam2body; }

  Eigen::Quaterniond imu_to_center_rotation() const { return q_imu2body; }

  Eigen::Vector3d imu_to_center_translation() const { return p_imu2body; }

  Eigen::Matrix3d imu_gyro_white_noise() const { return gyro_noise; }

  Eigen::Matrix3d imu_accel_white_noise() const { return accl_noise; }

  Eigen::Matrix3d imu_gyro_random_walk() const { return gyro_random_walk; }

  Eigen::Matrix3d imu_accel_random_walk() const { return accl_random_walk; }

  size_t init_map_frames() const { return 6; }

  size_t max_init_raw_frames() const { return (init_map_frames() - 1) * 3 + 1; }

  size_t min_init_raw_frames() const { return (init_map_frames() - 1) * 2 + 1; }

private:
  Eigen::Matrix3d K;
  Eigen::Quaterniond q_cam2body;
  Eigen::Vector3d p_cam2body;
  Eigen::Quaterniond q_imu2body;
  Eigen::Vector3d p_imu2body;
  Eigen::Matrix3d gyro_noise;
  Eigen::Matrix3d gyro_random_walk;
  Eigen::Matrix3d accl_noise;
  Eigen::Matrix3d accl_random_walk;
};

}} // namespace beam_models::camera_to_camera