#include <lidar_aggregation/lidar_aggregation_time_publisher_nodelet.h>

#include <stdlib.h>
#include <string>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/file.hpp>

#include <beam_utils/optional.h>
#include <beam_utils/pointclouds.h>

#include <lidar_aggregation/lidar_aggregators/end_time_lidar_aggregator.h>

namespace lidar_aggregation {

LidarAggregationTimePublisherNodelet::LidarAggregationTimePublisherNodelet() {}

void LidarAggregationTimePublisherNodelet::onInit() {
  nh_ = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();
  LoadParams();

  time_publisher_ =
      nh_.advertise<std_msgs::Time>(params_.aggregation_time_topic, 1000);
  odometry_subscriber_ = nh_.subscribe<nav_msgs::Odometry>(
      params_.odometry_topic, 100,
      boost::bind(&LidarAggregationTimePublisherNodelet::OdometryCallback, this,
                  _1));

  // Set nodelet's internal logger
  SetInternalLogger();
}

void LidarAggregationTimePublisherNodelet::LoadParams() {
  if (nh_.getParam("lidar_aggregation/aggregation_time_topic",
                   params_.aggregation_time_topic)) {
    ROS_INFO("Loaded parameter aggregation_time_topic: %s",
             params_.aggregation_time_topic.c_str());
  } else {
    ROS_ERROR("Could not load parameter aggregation_time_topic, using default: "
              "default_topic");
    throw std::invalid_argument{
        "Could not load parameter aggregation_time_topic"};
  }

  if (nh_.getParam("lidar_aggregation/odometry_topic",
                   params_.odometry_topic)) {
    ROS_INFO("Loaded parameter odometry_topic: %s", params_.odometry_topic.c_str());
  } else {
    ROS_ERROR("Could not load parameter odometry_topic");
    throw std::invalid_argument{"Could not load parameter odometry_topic"};
  }

  if (nh_.getParam("lidar_aggregation/aggregation_time",
                   params_.aggregation_time)) {
    ROS_INFO("Loaded parameter aggregation_time: %.2f", params_.aggregation_time);
  } else {
    params_.aggregation_time = 1;
    ROS_INFO("Could not load parameter lidar_frame, using: 1");
  }

  if (nh_.getParam("lidar_aggregation/log_directory", params_.log_directory)) {
    ROS_INFO("Loaded parameter log_directory: %s", params_.log_directory.c_str());
  } else {
    params_.log_directory = ".ros/log/";
    ROS_INFO("Could not parameter load log_directory, using .ros/log");
  }
}

// Set a logger to catch the errors published by the internal C++ library so
// that you don't spam the console!
void LidarAggregationTimePublisherNodelet::SetInternalLogger() {
  boost::log::add_common_attributes();

  // Set the logging directory
  // TODO(msmart) clean this up so that it doesn't use '/' convention for
  // files going to HOME. Maybe check for first character in parameter being
  // '/' and then treat as absolute, or relative if '/' is not first
  // character.
  std::string log_path = getenv("HOME") + params_.log_directory;

  // TODO(msmart) add more complicated example demonstrating tiers of severity
  // logging. For now - just drop anything less than info.

  // Filter based on logging severity
  boost::log::core::get()->set_filter(
      // trace and debug level log events are filtered out
      boost::log::trivial::severity >= boost::log::trivial::info);

  // Define the desired log file.
  // TODO(msmart) - fix this formatting so lint stops yelling.
  boost::log::add_file_log(
      // Set log file name
      boost::log::keywords::file_name =
          log_path + "/lidar_aggregation_nodelet_%N.log",
      // Set log file rotation size in bytes.
      // These values are low here for example.
      boost::log::keywords::rotation_size = 512,
      // Set log entry format
      boost::log::keywords::format = "[%TimeStamp%]: %Message%"
      // Call make_collector off of the sink created by add_file_log
      )
      ->locked_backend()
      ->set_file_collector(
          // Collectors are only applied after a log file is closed.
          // Each time a log file is closed, the collector checks to
          // see if an action needs to be taken relative to the next
          // log file.
          boost::log::sinks::file::make_collector(
              // 'target' sets the folder that will be "managed"
              // by overwriting logs to maintain following
              // objective.
              boost::log::keywords::target = log_path,
              // If the logs being created in total exceed
              // max_size, then the next log file created will
              // overwrite the first log file.
              boost::log::keywords::max_size = 5 * 512));
}

void LidarAggregationTimePublisherNodelet::OdometryCallback(
    const nav_msgs::OdometryConstPtr message) {
  std_msgs::Time time;
  time.data = message->header.stamp;
  time_publisher_.publish(time);
}

} // namespace lidar_aggregation

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(lidar_aggregation::LidarAggregationTimePublisherNodelet,
                       nodelet::Nodelet);
