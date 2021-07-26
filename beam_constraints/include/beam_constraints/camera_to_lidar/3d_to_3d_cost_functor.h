#pragma once

#include <fuse_core/eigen.h>
#include <fuse_core/macros.h>
#include <fuse_core/util.h>

#include <ceres/autodiff_cost_function.h>
#include <ceres/cost_function_to_functor.h>

namespace beam_constraints {

class 3dTo3DCostFunctor {
 public:
  FUSE_MAKE_ALIGNED_OPERATOR_NEW();

  /**
   * @brief Construct a cost function instance
   *
   * @param lidar_point the corresponding lidar point to the landmark, expressed
   * in the world or map frame (same frame as the landmark)
   */
  3dTo3DCostFunctor(const Eigen::Vector3d& lidar_point)
      : lidar_point_(lidar_point) {}

  template <typename T>
  bool operator()(const T* const landmark, T* residual) const {
    Eigen::Matrix<T, 3, 1> diff;
    diff(0, 0) = landmark[0] - std::static_cast<T>(lidar_point_[0]);
    diff(1, 0) = landmark[1] - std::static_cast<T>(lidar_point_[1]);
    diff(2, 0) = landmark[2] - std::static_cast<T>(lidar_point_[2]);

    residual[0] = diff.norm();
    return true;
  }

 private:
  Eigen::Vector3d lidar_point_;
};

}  // namespace beam_constraints
