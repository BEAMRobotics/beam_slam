#pragma once

#include <boost/lexical_cast.hpp>
#include <ros/node_handle.h>

#include <nlohmann/json.hpp>
#include <beam_utils/filesystem.h>

namespace bs_parameters {

struct ParameterBase {
  ParameterBase() = default;
  virtual ~ParameterBase() = default;

  /**
   * @brief Method for loading parameter values from ROS.
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  virtual void loadFromROS(const ros::NodeHandle& nh) = 0;
};

/**
 * @brief Utility method for handling non-required ROS params
 * @param[in] nh - The ROS node handle with which to load parameters
 * @param[in] key - The ROS parameter key for the required parameter
 * @param[out] value - The ROS parameter value for the \p key
 * @param[in] default value
 */
template <typename T>
void getParam(const ros::NodeHandle& nh, const std::string& key, T& value,
              const T& default_value) {
  if (!nh.getParam(key, value)) {
    value = default_value;
    const std::string info =
        "Could not find parameter " + key + " in namespace " +
        nh.getNamespace() +
        ", using default: " + boost::lexical_cast<std::string>(value);
    ROS_INFO_STREAM(info);
  }
}

/**
 * @brief Utility method for handling required ROS params
 * @param[in] nh - The ROS node handle with which to load parameters
 * @param[in] key - The ROS parameter key for the required parameter
 * @param[out] value - The ROS parameter value for the \p key
 * @throws std::runtime_error if the parameter does not exist
 */
template <typename T>
void getParamRequired(const ros::NodeHandle& nh, const std::string& key,
                      T& value) {
  if (!nh.getParam(key, value)) {
    const std::string error = "Could not find required parameter " + key +
                              " in namespace " + nh.getNamespace();
    ROS_FATAL_STREAM(error);
    throw std::runtime_error(error);
  }
}

/**
 * @brief Utility method for handling required ROS params
 * @param[in] J - json object containing the parameter
 * @param[in] key - The parameter key for the required parameter
 * @param[out] value - The parameter value for the \p key
 * @param[in] default value
 */
template <typename T>
void getParamJson(const nlohmann::json& J, const std::string& key, T& value,
                  const T& default_value) {
  try {
    beam::ValidateJsonKeysOrThrow({key}, J);
    value = J[key];
  } catch (...) {
    value = default_value;
    const std::string info =
        "Could not find parameter " + key + " in JSON file" +
        ", using default: " + boost::lexical_cast<std::string>(value);
    ROS_INFO_STREAM(info);
  }
}

} // namespace bs_parameters
