#pragma once

#include <queue>

#include <fuse_core/async_sensor_model.h>
#include <fuse_core/fuse_macros.h>
#include <fuse_core/throttled_callback.h>
#include <fuse_graphs/hash_graph.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Time.h>

#include <bs_common/bs_msgs.h>
#include <bs_common/extrinsics_lookup_online.h>
#include <bs_models/frame_initializers/frame_initializers.h>
#include <bs_models/imu/imu_preintegration.h>
#include <bs_parameters/models/calibration_params.h>
#include <bs_parameters/models/inertial_odometry_params.h>

namespace bs_models {

struct ImuConstraintData {
  fuse_core::UUID constraint_uuid;
  std::vector<sensor_msgs::Imu::ConstPtr> imu_data;
};

/**
 * Store all unused IMU data in an IMU buffer and when a constraint gets added,
 * we move that data to the constraint buffer.
 */
class ImuBuffer {
public:
  explicit ImuBuffer(double buffer_length_s = 5);

  // add IMU data to raw IMU buffer
  void AddData(const sensor_msgs::Imu::ConstPtr& msg);

  // move data from raw IMU buffer to constraint buffer up until time_ns
  void SetConstraint(int64_t time_ns, const fuse_core::UUID& constraint_uuid);

  // directly add constraint data to the constraint buffer
  void AddConstraintData(int64_t time_ns, ImuConstraintData);

  // extract all imu constraint data after a specific time. This will remove it
  // from the constraint buffer and therefore will need to be re-added
  std::vector<ImuConstraintData>
      ExtractConstraintDataAfterTime(int64_t time_ns) const;

  int64_t GetPriorConstraintTime(int64_t time_ns) const;

  int64_t GetNextConstraintTime(int64_t time_ns) const;

  int64_t GetLastConstraintTime() const;

private:
  void CleanOverflow();

  // maps constraint timestamp to constraint data. Constraint timestamp
  // should match the end of the IMU data in the constraint
  std::map<int64_t, ImuConstraintData> constraint_buffer_;

  // raw IMU data not added to the constraint buffer
  std::map<int64_t, sensor_msgs::Imu::ConstPtr> imu_buffer_;
  int64_t buffer_length_ns_;
};

class InertialOdometry : public fuse_core::AsyncSensorModel {
public:
  FUSE_SMART_PTR_DEFINITIONS(InertialOdometry);

  /**
   * @brief Default Constructor
   */
  InertialOdometry();

  /**
   * @brief Default Destructor
   */
  ~InertialOdometry() override = default;

private:
  /**
   * @brief Callback for imu processing, this will make sure the imu messages
   * are added to the buffer at the correct time
   * @param[in] msg - The imu msg to process
   */
  void processIMU(const sensor_msgs::Imu::ConstPtr& msg);

  /**
   * @brief Callback for processing a Time message which serves as a trigger to
   * add IMU constraints
   * @param[in] msg - The time msg to process
   */
  void processTrigger(const std_msgs::Time::ConstPtr& msg);

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
   * @brief Subscribe to the input topics to start sending transactions to the
   * optimizer
   */
  void onStart() override;

  /**
   * @brief Unsubscribe to the input topics
   */
  void onStop() override {}

  /**
   * @brief Read in graph updates
   */
  void onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph_msg) override;

  /// @brief Computes relative motion and publishes to odometry
  /// @param prev_stamp
  /// @param curr_stamp
  void ComputeRelativeMotion(const ros::Time& prev_stamp,
                             const ros::Time& curr_stamp);

  /// @brief Computes pose in world frame wrt the graph and published to
  /// odometry
  /// @param curr_stamp
  void ComputeAbsolutePose(const ros::Time& curr_stamp);

  int odom_seq_ = 0;
  bool initialized_{false};
  ros::Time prev_stamp_{0.0};
  Eigen::Matrix4d T_ODOM_IMUprev_{Eigen::Matrix4d::Identity()};

  // calibration parameters
  bs_parameters::models::CalibrationParams calibration_params_;

  // loadable parameters
  bs_parameters::models::InertialOdometryParams params_;

  // subscribers
  ros::Subscriber imu_subscriber_;
  ros::Subscriber trigger_subscriber_;

  // publishers
  ros::Publisher odometry_publisher_;
  ros::Publisher pose_publisher_;

  // data storage
  std::queue<sensor_msgs::Imu::ConstPtr> imu_buffer_;

  // primary odom objects
  std::shared_ptr<ImuPreintegration> imu_preint_;
  bs_models::ImuPreintegration::Params imu_params_;
  std::unique_ptr<frame_initializers::FrameInitializerBase> frame_initializer_;

  // extrinsics
  bs_common::ExtrinsicsLookupOnline& extrinsics_ =
      bs_common::ExtrinsicsLookupOnline::GetInstance();

  // throttled callbacks for imu
  using ThrottledIMUCallback =
      fuse_core::ThrottledMessageCallback<sensor_msgs::Imu>;
  ThrottledIMUCallback throttled_imu_callback_;

  // throttle callback for odometry
  using ThrottledTriggerCallback =
      fuse_core::ThrottledMessageCallback<std_msgs::Time>;
  ThrottledTriggerCallback throttled_trigger_callback_;
};

} // namespace bs_models
