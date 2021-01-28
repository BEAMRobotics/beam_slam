#include <string>

#include <nodelet/loader.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "lidar_aggregation_time_publisher_node");
  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  std::string nodelet_name = ros::this_node::getName();
  nodelet.load(nodelet_name,
               "lidar_aggregation/lidar_aggregation_time_publisher_nodelet",
               remap, nargv);
  ros::spin();
  return 0;
}
