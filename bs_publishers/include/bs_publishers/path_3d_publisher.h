#pragma once

#include <fuse_core/async_publisher.h>
#include <fuse_core/fuse_macros.h>
#include <fuse_core/graph.h>
#include <fuse_core/transaction.h>
#include <ros/ros.h>

namespace bs_publishers {

/**
 * @brief Publisher plugin that publishes all of the stamped 3D poses as a
 * nav_msgs::Path message.
 *
 * Parameters:
 *  - device_id (uuid string, default: 00000000-0000-0000-0000-000000000000) The
 * device/robot ID to publish
 *  - device_name (string) Used to generate the device/robot ID if the device_id
 * is not provided
 *  - frame_id (string, default: map)  Name for the robot's map frame
 */
class Path3DPublisher : public fuse_core::AsyncPublisher {
public:
  FUSE_SMART_PTR_DEFINITIONS(Path3DPublisher);

  /**
   * @brief Constructor
   */
  Path3DPublisher();

  /**
   * @brief Destructor
   */
  virtual ~Path3DPublisher() = default;

  /**
   * @brief Perform any required post-construction initialization, such as
   * advertising publishers or reading from the parameter server.
   */
  void onInit() override;

  void onStart() override;

  /**
   * @brief Notify the publisher about variables that have been added or removed
   *
   * @param[in] transaction A Transaction object, describing the set of
   * variables that have been added and/or removed
   * @param[in] graph       A read-only pointer to the graph object, allowing
   * queries to be performed whenever needed
   */
  void notifyCallback(fuse_core::Transaction::ConstSharedPtr transaction,
                      fuse_core::Graph::ConstSharedPtr graph) override;

protected:
  fuse_core::UUID device_id_;     //!< The UUID of the device to be published
  std::string frame_id_;          //!< The name of the frame for this path
  ros::Publisher path_publisher_; //!< The publisher that sends the entire robot
                                  //!< trajectory as a path
  ros::Publisher pose_array_publisher_; //!< The publisher that sends the entire
                                        //!< robot trajectory as a pose array
};

} // namespace bs_publishers
