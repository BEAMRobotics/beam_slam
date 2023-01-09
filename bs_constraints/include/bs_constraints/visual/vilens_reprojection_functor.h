#ifndef FUSE_MODELS_VILENS_VISUAL_COST_FUNCTOR_H
#define FUSE_MODELS_VILENS_VISUAL_COST_FUNCTOR_H

#include <fuse_core/eigen.h>
#include <fuse_core/fuse_macros.h>
#include <fuse_core/util.h>

#include <beam_calibration/CameraModel.h>
#include <beam_cv/Utils.h>
#include <beam_optimization/CamPoseReprojectionCost.h>
#include <beam_utils/math.h>

#include <ceres/autodiff_cost_function.h>
#include <ceres/cost_function_to_functor.h>
#include <ceres/numeric_diff_cost_function.h>
#include <ceres/rotation.h>

namespace fuse_constraints {

class VilensReprojectionFunctor {
public:
  FUSE_MAKE_ALIGNED_OPERATOR_NEW();

  /**
   * @brief Construct a cost function instance
   *
   * @param[in] pixel_measurement The pixel location of feature in the image
   * @param[in] cam_model The camera intrinsics for projection
   */
  VilensReprojectionFunctor(
      const Eigen::Vector2d& pixel_measurement,
      const std::shared_ptr<beam_calibration::CameraModel> cam_model,
      const Eigen::Matrix4d& T_cam_baselink)
      : pixel_measurement_(pixel_measurement),
        cam_model_(cam_model),
        T_cam_baselink_(T_cam_baselink) {
    /*
    The logic behind this covariance is in VILENS:
    https://arxiv.org/abs/2107.07243.
    */
    // undistort pixel measurement
    Eigen::Vector2i pixel_i = pixel_measurement_.cast<int>();
    Eigen::Vector2i und_pixel;
    if (!cam_model_->UndistortPixel(pixel_i, und_pixel)) {
      ROS_FATAL_STREAM("Invalid pixel measurement for visual factor, "
                       "undistorted pixel is in not image domain.");
      throw std::runtime_error{"Invalid pixel measurement for visual factor, "
                               "undistorted pixel is in not image domain."};
    }
    undistorted_pixel_measurement_ = und_pixel.cast<double>();

    float sigma = 2;
    // get circle around pixel in distorted image
    std::vector<Eigen::Vector2i> circle =
        beam_cv::GetCircle(pixel_i, int(sigma));

    // undistort circle to get a covariance estimate
    std::vector<Eigen::Vector2d> circle_d;
    for (auto& p : circle) {
      Eigen::Vector2i und_p;
      if (cam_model_->UndistortPixel(p, und_p)) {
        circle_d.push_back(und_p.cast<double>());
      }
    }

    // if not enough points to fit ellipse, then we use a very large covariance
    // to to the assumption that its very close to the edge
    try {
      A_ = beam_cv::FitEllipse(circle_d);
    } catch (const std::runtime_error& re) {
      A_ = Eigen::Matrix2d::Identity();
      A_(0, 0) = std::pow(3 * sigma, 2);
      A_(1, 1) = std::pow(3 * sigma, 2);
    }

    // projection functor
    compute_projection.reset(new ceres::CostFunctionToFunctor<2, 3>(
        new ceres::NumericDiffCostFunction<
            beam_optimization::CameraProjectionFunctor, ceres::CENTRAL, 2, 3>(
            new beam_optimization::CameraProjectionFunctor(
                cam_model_, pixel_measurement_))));
  }

  template <typename T>
  bool operator()(const T* const R_WORLD_BASELINK,
                  const T* const t_WORLD_BASELINK, const T* const P_WORLD,
                  T* residual) const {
    // transform point from world frame into camera frame
    Eigen::Matrix<T, 4, 4> T_CAM_BASELINK = T_cam_baselink_.cast<T>();

    T R_WORLD_BASELINK_mat[9];
    ceres::QuaternionToRotation(R_WORLD_BASELINK, R_WORLD_BASELINK_mat);

    Eigen::Matrix<T, 4, 4> T_WORLD_BASELINK;
    T_WORLD_BASELINK(0, 0) = R_WORLD_BASELINK_mat[0];
    T_WORLD_BASELINK(0, 1) = R_WORLD_BASELINK_mat[1];
    T_WORLD_BASELINK(0, 2) = R_WORLD_BASELINK_mat[2];
    T_WORLD_BASELINK(0, 3) = t_WORLD_BASELINK[0];
    T_WORLD_BASELINK(1, 0) = R_WORLD_BASELINK_mat[3];
    T_WORLD_BASELINK(1, 1) = R_WORLD_BASELINK_mat[4];
    T_WORLD_BASELINK(1, 2) = R_WORLD_BASELINK_mat[5];
    T_WORLD_BASELINK(1, 3) = t_WORLD_BASELINK[1];
    T_WORLD_BASELINK(2, 0) = R_WORLD_BASELINK_mat[6];
    T_WORLD_BASELINK(2, 1) = R_WORLD_BASELINK_mat[7];
    T_WORLD_BASELINK(2, 2) = R_WORLD_BASELINK_mat[8];
    T_WORLD_BASELINK(2, 3) = t_WORLD_BASELINK[2];
    T_WORLD_BASELINK(3, 0) = (T)0;
    T_WORLD_BASELINK(3, 1) = (T)0;
    T_WORLD_BASELINK(3, 2) = (T)0;
    T_WORLD_BASELINK(3, 3) = (T)1;

    Eigen::Matrix<T, 4, 1> P_WORLD_h;
    P_WORLD_h[0] = P_WORLD[0];
    P_WORLD_h[1] = P_WORLD[1];
    P_WORLD_h[2] = P_WORLD[2];
    P_WORLD_h[3] = (T)1;

    Eigen::Matrix<T, 4, 1> P_BASELINK_h =
        T_WORLD_BASELINK.inverse() * P_WORLD_h;
    Eigen::Matrix<T, 3, 1> P_CAM =
        (T_CAM_BASELINK * P_BASELINK_h).hnormalized();
    T P_CAMERA[3];
    P_CAMERA[0] = P_CAM[0];
    P_CAMERA[1] = P_CAM[1];
    P_CAMERA[2] = P_CAM[2];

    const T* P_CAMERA_const = &(P_CAMERA[0]);

    // project point into pixel space
    T pixel_projected[2];
    (*compute_projection)(P_CAMERA_const, &(pixel_projected[0]));

    // compute the reprojection residual
    Eigen::Matrix<T, 2, 1> result;
    result[0] = (pixel_measurement_.cast<T>()[0] - pixel_projected[0]);
    result[1] = (pixel_measurement_.cast<T>()[1] - pixel_projected[1]);

    // apply sqrt information matrix
    result = A_.cast<T>() * result;

    // fill residual
    residual[0] = result[0];
    residual[1] = result[1];
    return true;
  }

private:
  Eigen::Matrix2d A_;
  Eigen::Vector2d pixel_measurement_;
  Eigen::Vector2d undistorted_pixel_measurement_;
  std::shared_ptr<beam_calibration::CameraModel> cam_model_;
  std::unique_ptr<ceres::CostFunctionToFunctor<2, 3>> compute_projection;
  Eigen::Matrix4d T_cam_baselink_;
};

} // namespace fuse_constraints

#endif // FUSE_MODELS_VILENS_VISUAL_COST_FUNCTOR_H
