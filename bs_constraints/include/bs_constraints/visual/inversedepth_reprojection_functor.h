#pragma once

#include <fuse_core/eigen.h>
#include <fuse_core/fuse_macros.h>
#include <fuse_core/util.h>

#include <bs_constraints/helpers.h>

#include <beam_cv/Utils.h>
#include <beam_utils/math.h>
#include <beam_utils/se3.h>

namespace bs_constraints {

class InverseDepthReprojectionFunctor {
public:
  FUSE_MAKE_ALIGNED_OPERATOR_NEW();

  /**
   * @brief Construct a cost function instance for an inverse depth reprojection
   *
   * @param[in] information_matrix Residual weighting matrix
   * @param[in] pixel_measurement Pixel measurement
   * @param[in] intrinsic_matrix Camera intrinsic matrix (K):
   * [fx, 0, cx]
   * [0, fy, cy]
   * [0,  0,  1]
   * @param[in] T_cam_baselink Camera extrinsic
   * @param[in] bearing Bearing vector of the inverse depth landmark [mx, my, 1]
   */
  InverseDepthReprojectionFunctor(const Eigen::Matrix2d& information_matrix,
                                  const Eigen::Vector2d& pixel_measurement,
                                  const Eigen::Matrix3d& intrinsic_matrix,
                                  const Eigen::Matrix4d& T_cam_baselink,
                                  const Eigen::Vector3d& bearing)
      : information_matrix_(information_matrix),
        pixel_measurement_(pixel_measurement),
        intrinsic_matrix_(intrinsic_matrix),
        T_cam_baselink_(T_cam_baselink),
        bearing_(bearing) {}

  /**
   * @brief Construct a cost function instance for an inverse depth reprojection
   * An inverse depth reprojection cost function is defined by two poses: the
   * anchor pose of the inverse depth landmark, the measurement pose and the
   * inverse depth landmark itself. The anchor pose is used to determine the
   * euclidean location of the landmark in the world frame, which then gets
   * projected into the measurement frame and the normal reprojection is
   * computed.
   *
   * This transformation can be done by finding the relative transform between
   * anchor and measurement frame and constructing a projection matrix:
   * P = [K]*[R|t], where [R|t] is the 3x4 matrix representing the
   * transformation from the anchor frame to the measurement frame.
   */
  template <typename T>
  bool operator()(const T* const o_WORLD_BASELINKa,
                  const T* const p_WORLD_BASELINKa,
                  const T* const o_WORLD_BASELINKm,
                  const T* const p_WORLD_BASELINKm,
                  const T* const inverse_depth, T* residual) const {
    // get extrinsic
    Eigen::Matrix<T, 4, 4> T_BASELINK_CAM =
        beam::InvertTransform(T_cam_baselink_).cast<T>();

    // get anchor pose as 4x4 matrix
    Eigen::Matrix<T, 4, 4> T_WORLD_BASELINKa =
        bs_constraints::OrientationAndPositionToTransformationMatrix(
            o_WORLD_BASELINKa, p_WORLD_BASELINKa);
    Eigen::Matrix<T, 4, 4> T_WORLD_CAMERAa = T_WORLD_BASELINKa * T_BASELINK_CAM;

    // get measurement pose as 4x4 matrix
    Eigen::Matrix<T, 4, 4> T_WORLD_BASELINKm =
        bs_constraints::OrientationAndPositionToTransformationMatrix(
            o_WORLD_BASELINKm, p_WORLD_BASELINKm);
    Eigen::Matrix<T, 4, 4> T_WORLD_CAMERAm = T_WORLD_BASELINKm * T_BASELINK_CAM;

    // get relative pose between anchor and measurement
    Eigen::Matrix<T, 4, 4> T_CAMERAm_CAMERAa =
        bs_constraints::InvertTransform(T_WORLD_CAMERAm) * T_WORLD_CAMERAa;

    // ! Method 1:
    // const T depth = static_cast<T>(1.0) / inverse_depth[0];
    // Eigen::Matrix<T, 3, 1> anchor_camera_t_point = depth *
    // bearing_.cast<T>();

    // Eigen::Matrix<T, 3, 1> measurement_camera_t_point =
    //     (T_CAMERAm_CAMERAa *
    //     anchor_camera_t_point.homogeneous()).hnormalized();

    // // project into measurement image
    // Eigen::Matrix<T, 2, 1> reproj =
    //     (intrinsic_matrix_.cast<T>() * measurement_camera_t_point)
    //         .hnormalized();

    // ! Method 2:
    /* The projection matrix computed here can be broken down into:
    P = K[R|t], where [R|t] is the 3x4 matrix representing the transformation
    from the anchor frame to the measurement frame. Typically, to transform and
    project a point in one camera frame to another we would multiply this matrix
    by the homogeneous representation of the point in the first frame. However
    if we leave the point represented as (mx, my, mz, 1/d) where [mx, my, mz] is
    the unit direction vector to the landmark from the first frame, and 1/d is
    the inverse depth, then it is equivalent to the prior explanation since the
    point will be normalized by the z component, which in this case is the
    inverse depth (i.e. by dividing by the inverse depth, we multiply by the
    actual depth). Multiplying the unit direction vector by the depth gets the
    euclidean location of the point. */

    // create projection matrix
    const Eigen::Matrix<T, 3, 4> projection_matrix =
        intrinsic_matrix_.cast<T>() * T_CAMERAm_CAMERAa.block(0, 0, 3, 4);

    // compute the inverse depth and bearing vector (mx, my, mz, 1/d)
    Eigen::Matrix<T, 4, 1> bearing_and_inversedepth;
    bearing_and_inversedepth << bearing_.cast<T>(), inverse_depth[0];

    // project into measurement image
    Eigen::Matrix<T, 2, 1> reproj =
        (projection_matrix * bearing_and_inversedepth).hnormalized();

    residual[0] = static_cast<T>(pixel_measurement_[0]) - reproj[0];
    residual[1] = static_cast<T>(pixel_measurement_[1]) - reproj[1];
    Eigen::Map<Eigen::Matrix<T, 2, 1>> residual_map(residual);
    residual_map.applyOnTheLeft(information_matrix_.template cast<T>());

    return true;
  }

private:
  Eigen::Matrix2d information_matrix_; //!< The residual weighting matrix
  Eigen::Vector2d pixel_measurement_;  //!< The measured pixel value
  Eigen::Vector3d bearing_;
  Eigen::Matrix3d intrinsic_matrix_;
  Eigen::Matrix4d T_cam_baselink_;
};

} // namespace bs_constraints
