#pragma once

#include <boost/algorithm/string/case_conv.hpp>
#include <ros/console.h>

#include <beam_common/variable_traits.h>

namespace beam_common {

/**
 * @brief Utility method for printing and throwing an error for invalid
 * dimension specification
 * @param[in] dimension - The erroneous dimension name
 * @throws runtime_error
 */
inline void throwDimensionError(const std::string& dimension) {
  std::string error = "Dimension " + dimension + " is not valid for this type.";
  ROS_ERROR_STREAM(error);
  throw std::runtime_error(error);
}

template <typename T>
typename std::enable_if<is_linear_3d<T>::value, size_t>::type
    toIndex(const std::string& dimension) {
  auto lower_dim = boost::algorithm::to_lower_copy(dimension);
  if (lower_dim == "x") return static_cast<size_t>(T::X);
  if (lower_dim == "y") return static_cast<size_t>(T::Y);
  if (lower_dim == "z") return static_cast<size_t>(T::Z);

  throwDimensionError(dimension);

  return 0u;
}

template <typename T>
typename std::enable_if<is_angular_3d<T>::value, size_t>::type
    toIndex(const std::string& dimension) {
  auto lower_dim = boost::algorithm::to_lower_copy(dimension);
  if (lower_dim == "roll" || lower_dim == "r")
    return static_cast<size_t>(
        fuse_variables::Orientation3DStamped::Euler::ROLL);
  if (lower_dim == "pitch" || lower_dim == "p")
    return static_cast<size_t>(
        fuse_variables::Orientation3DStamped::Euler::PITCH);
  if (lower_dim == "yaw" || lower_dim == "y")
    return static_cast<size_t>(
        fuse_variables::Orientation3DStamped::Euler::YAW);

  throwDimensionError(dimension);

  return 0u;
}

/**
 * @brief Utility method to convert a vector of dimension names to a vector of
 * dimension indices
 *
 * Note that the dimensions are sorted, and so the order in which the user
 * specifies them will have no bearing when the measurement vectors and
 * covariances are actually built elsewhere.
 *
 * @param[in] dimension_names - The dimension names to convert
 * @return a vector of indices that are consistent with the enumerations for
 * that variable type
 * @throws runtime_error if any dimension name is invalid
 */
template <typename T>
std::vector<size_t>
    getDimensionIndices(const std::vector<std::string>& dimension_names) {
  std::vector<size_t> indices;
  indices.reserve(dimension_names.size());

  std::transform(dimension_names.begin(), dimension_names.end(),
                 std::back_inserter(indices), toIndex<T>);

  // Remove duplicates
  std::sort(indices.begin(), indices.end());
  indices.erase(std::unique(indices.begin(), indices.end()), indices.end());

  return indices;
}

} // namespace beam_common
