#pragma once

#include <list>

#include <fuse_constraints/relative_pose_3d_stamped_constraint.h>
#include <fuse_core/async_sensor_model.h>
#include <fuse_core/uuid.h>
#include <sensor_msgs/PointCloud2.h>

#include <beam_utils/pointclouds.h>
#include <beam_matching/Matcher.h>

#include <beam_parameters/models/scan_matcher_params.h>

namespace beam_models { namespace frame_to_frame {

struct ReferenceCloud{
  PoinCloudPtr cloud;
  ros::Time time;
  Eigen::Matrix4d T_REF_CLOUD;
};

class ScanMatcher : public fuse_core::AsyncSensorModel {
  using Pose3D = fuse_variables::Position3DStamped;

public:
  SMART_PTR_DEFINITIONS(ScanMatcher);
  using ParameterType = beam_parameters::models::ScanMatcherParams;

  ScanMatcher();

  ~ScanMatcher() override = default;

  /**
   * @brief Callback for pose messages
   * @param[in] msg - The pose message to process
   */
  void process(const sensor_msgs::PointCloud2::ConstPtr& msg);

protected:
  /**
   * @brief Perform any required initialization for the sensor model
   *
   * This could include things like reading from the parameter server or
   * subscribing to topics. The class's node handles will be properly
   * initialized before SensorModel::onInit() is called. Spinning of the
   * callback queue will not begin until after the call to SensorModel::onInit()
   * completes.
   */
  void onInit() override;

  /**
   * @brief Subscribe to the input topic to start sending transactions to the
   * optimizer
   */
  void onStart() override;

  /**
   * @brief Unsubscribe from the input topic to stop sending transactions to the
   * optimizer
   */
  void onStop() override;

  void onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph_msg) override;

  fuse_core::UUID device_id_; //!< The UUID of this device
  ParameterType params_;
  ros::Subscriber pointcloud_subscriber_;
  std::list<ReferenceCloud> reference_clouds_;
  std::unique_ptr<beam_matching::Matcher> matcher_;

  using PointCloudThrottledCallback = common::ThrottledCallback<sensor_msgs::PointCloud2>;
  PointCloudThrottledCallback throttled_callback_;
};

}} // namespace beam_models::frame_to_frame
