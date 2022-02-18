#include <bs_models/frame_initializers/frame_initializer_base.h>
#include <nlohmann/json.hpp>

#include <bs_models/frame_initializers/frame_initializers.h>

#include <beam_utils/filesystem.h>

namespace bs_models { namespace frame_initializers {

std::unique_ptr<frame_initializers::FrameInitializerBase>
    FrameInitializerBase::Create(const std::string& config_path) {
  std::unique_ptr<frame_initializers::FrameInitializerBase> frame_initializer;
  // Load frame ids
  nlohmann::json J;
  if (!beam::ReadJson(config_path, J)) {
    throw std::runtime_error{"Invalid frame initializer config path"};
  }

  ROS_INFO_STREAM("Loading frame initializer from: " << config_path);

  std::string type;
  std::string info;
  std::string sensor_frame_id_override;
  std::vector<double> tf_override;
  int queue_size;
  int64_t poses_buffer_time;

  try {
    type = J["type"];
  } catch (...) {
    ROS_ERROR("Missing or misspelt parameter: 'type'");
    throw std::runtime_error{"Missing or misspelt parameter: 'type'"};
  }

  try {
    info = J["info"];
  } catch (...) {
    ROS_ERROR("Missing or misspelt parameter: 'info'");
    throw std::runtime_error{"Missing or misspelt parameter: 'info'"};
  }

  try {
    sensor_frame_id_override = J["sensor_frame_id_override"];
  } catch (...) {
    ROS_ERROR("Missing or misspelt parameter: 'sensor_frame_id_override'");
    throw std::runtime_error{
        "Missing or misspelt parameter: 'sensor_frame_id_override'"};
  }

  try {
    for (const auto& value : J["T_ORIGINAL_OVERRIDE"]) {
      tf_override.push_back(value.get<double>());
    }
  } catch (...) {
    ROS_ERROR("Missing or misspelt parameter: 'T_ORIGINAL_OVERRIDE'");
    throw std::runtime_error{
        "Missing or misspelt parameter: 'T_ORIGINAL_OVERRIDE'"};
  }

  try {
    queue_size = J["queue_size"];
  } catch (...) {
    ROS_ERROR("Missing or misspelt parameter: 'queue_size'");
    throw std::runtime_error{"Missing or misspelt parameter: 'queue_size'"};
  }

  try {
    poses_buffer_time = J["poses_buffer_time"];
  } catch (...) {
    ROS_ERROR("Missing or misspelt parameter: 'poses_buffer_time'");
    throw std::runtime_error{
        "Missing or misspelt parameter: 'poses_buffer_time'"};
  }

  if (type == "POSEFILE") {
    frame_initializer =
        std::make_unique<frame_initializers::PoseFileFrameInitializer>(info);

  } else {
    Eigen::Matrix4d T_ORIGINAL_OVERRIDE;
    if (tf_override.size() != 16) {
      ROS_WARN("Invalid T_ORIGINAL_OVERRIDE params, required 16 params, "
                "given: %d. Using default identity transform",
                tf_override.size());
      T_ORIGINAL_OVERRIDE = Eigen::Matrix4d::Identity();
    } else {
      T_ORIGINAL_OVERRIDE = Eigen::Matrix4d(tf_override.data());
    }

    if (type == "TRANSFORM") {
      frame_initializer =
          std::make_unique<frame_initializers::TransformFrameInitializer>(
              info, queue_size, poses_buffer_time, sensor_frame_id_override,
              T_ORIGINAL_OVERRIDE);
    } else if (type == "ODOMETRY") {
      frame_initializer =
          std::make_unique<frame_initializers::OdometryFrameInitializer>(
              info, queue_size, poses_buffer_time, sensor_frame_id_override,
              T_ORIGINAL_OVERRIDE);
    } else {
      ROS_ERROR("Invalid frame initializer type.");
      throw std::runtime_error{"Invalid frame initializer type."};
    }
  }

  return std::move(frame_initializer);
}

}} // namespace bs_models::frame_initializers