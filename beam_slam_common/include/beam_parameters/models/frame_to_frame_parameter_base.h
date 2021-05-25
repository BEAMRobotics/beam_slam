#pragma once

#include <boost/lexical_cast.hpp>
#include <ros/node_handle.h>

#include <beam_common/sensor_config.h>

#include <beam_parameters/parameter_base.h>

namespace beam_parameters { namespace models {

struct FrameToFrameParameterBase : public ParameterBase {
public:
  /**
   * @brief Method for loading parameter values from ROS. This virtual class
   * loads all params specific to the derived class which does not include the
   * default FrameToFrame params
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  virtual void loadExtraParams(const ros::NodeHandle& nh) = 0;

  /**
   * @brief Method for loading parameter values from ROS. This calls two
   * functions, one that loads the default FrameToFrame params, and one that
   * loads the extra params specific to the derived class (this must be defined
   * in the derived class)
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFromROS(const ros::NodeHandle& nh) final {
    loadExtraParams(nh);
    loadFrameToFrameDefaultParams(nh);
  }

  /**
   * @brief Method for loading parameter values from ROS. This loads the default
   * FrameToFrame params. Each frame to frame sensor model will require a
   * sensor_frame, topic to subscribe to for the sensor data, and a frame
   * initializer. To save code duplication, this id added to the params and
   * sensor model base classes
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFrameToFrameDefaultParams(const ros::NodeHandle& nh) {
    getParam<std::string>(nh, "frame_initializer_type", frame_initializer_type,
                          frame_initializer_type);
    getParam<std::string>(nh, "frame_initializer_info", frame_initializer_info,
                          frame_initializer_info);
    getParamRequired<std::string>(nh, "subscriber_topic", subscriber_topic);
  }
  
  std::string subscriber_topic;
  std::string frame_initializer_type{"ODOMETRY"};
  std::string frame_initializer_info{""};

};

}} // namespace beam_parameters::models
