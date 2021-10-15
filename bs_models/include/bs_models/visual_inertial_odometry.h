#pragma once

#include <queue>

#include <bs_common/bs_msgs.h>
#include <fuse_core/async_sensor_model.h>
#include <fuse_core/throttled_callback.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include <beam_calibration/CameraModel.h>
#include <beam_cv/geometry/PoseRefinement.h>
#include <beam_cv/trackers/Trackers.h>

#include <bs_common/extrinsics_lookup_online.h>
#include <bs_models/active_submap.h>
#include <bs_models/imu_preintegration.h>
#include <bs_models/vision/keyframe.h>
#include <bs_models/vision/vio_initialization.h>
#include <bs_models/vision/visual_map.h>
#include <bs_parameters/models/calibration_params.h>
#include <bs_parameters/models/camera_params.h>

namespace bs_models {

using namespace bs_common;
using namespace vision;

class VisualInertialOdometry : public fuse_core::AsyncSensorModel {
public:
  SMART_PTR_DEFINITIONS(VisualInertialOdometry);

  /**
   * @brief Default Constructor
   */
  VisualInertialOdometry();

  /**
   * @brief Default Destructor
   */
  ~VisualInertialOdometry() override = default;

  /**
   * @brief Callback for image processing, this callback will add visual
   * constraints and triangulate new landmarks when required
   * @param[in] msg - The image to process
   */
  void processImage(const sensor_msgs::Image::ConstPtr &msg);

  /**
   * @brief Callback for imu processing, this will make sure the imu messages
   * are added to the buffer at the correct time
   * @param[in] msg - The imu msg to process
   */
  void processIMU(const sensor_msgs::Imu::ConstPtr &msg);

protected:
  fuse_core::UUID device_id_; //!< The UUID of this device

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
   * @brief Unsubscribe to the input topic
   */
  void onStop() override {}

  /**
   * @brief Callback for when a newly optimized graph is available
   */
  void onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph_msg) override;

private:
  /**
   * @brief Localizes a given frame using the tracker and the current visual map
   * @param img_time time of image to localize
   * @return T_WORLD_BASELINK
   */
  Eigen::Matrix4d LocalizeFrame(const ros::Time &img_time);

  /**
   * @brief Determines if a frame is a keyframe
   * @param img_time time of image to determine if its a keyframe
   * @param T_WORLD_BASELINK initial odometry estimate
   * @return true or false decision
   */
  bool IsKeyframe(const ros::Time &img_time,
                  const Eigen::Matrix4d &T_WORLD_BASELINK);

  /**
   * @brief Extends the map at the current keyframe time and adds the visual
   * constraints
   */
  void ExtendMap();

  /**
   * @brief creates inertial cosntraint for the current keyframe and merges with
   * the input transaction
   * @param transaction current transaction to merge inertial constraints with
   * (typically built from extend map)
   */
  void AddInertialConstraint(fuse_core::Transaction::SharedPtr transaction);

  /**
 * @brief Copies all variables and constraints in the init graph and sends to
 * fuse optimizer
 * @param init_graph the graph obtained from the initializer
 */
  void SendInitializationGraph(const fuse_graphs::HashGraph &init_graph);

  /**
   * @brief Adds the keyframe pose to the graph and publishes keyframe header to
   * notify any lsiteners
   * @param T_WORLD_CAMERA pose of the keyframe to add
   */
  void NotifyNewKeyframe(const Eigen::Matrix4d &T_WORLD_BASELINK);

  /**
   * @brief Publishes the initial odometry estimate of frame
   * @param img_time time frame
   * @param T_WORLD_BASELINK initial odometry estimate
   */
  void PublishInitialOdometry(const ros::Time &img_time,
                              const Eigen::Matrix4d &T_WORLD_BASELINK);

  /**
   * @brief Publishes the oldest keyframe that is stored as a slam chunk message
   */
  void PublishSlamChunk();

  /**
   * @brief Publishes landmark ids
   * @param ids list of landmark ids
   */
  void PublishLandmarkIDs(const std::vector<uint64_t> &ids);

protected:
  // loadable camera parameters
  bs_parameters::models::CameraParams camera_params_;

  // calibration parameters
  bs_parameters::models::CalibrationParams calibration_params_;

  // subscribers
  ros::Subscriber image_subscriber_;
  ros::Subscriber imu_subscriber_;

  // publishers
  ros::Publisher init_odom_publisher_;
  ros::Publisher new_keyframe_publisher_;
  ros::Publisher slam_chunk_publisher_;
  ros::Publisher landmark_publisher_;
  ros::Publisher reloc_publisher_;

  // image and imu queues for proper synchronization
  std::queue<sensor_msgs::Image> image_buffer_;
  std::queue<sensor_msgs::Imu> imu_buffer_;

  // callbacks for messages
  using ThrottledImageCallback =
      fuse_core::ThrottledMessageCallback<sensor_msgs::Image>;
  using ThrottledIMUCallback =
      fuse_core::ThrottledMessageCallback<sensor_msgs::Imu>;
  ThrottledImageCallback throttled_image_callback_;
  ThrottledIMUCallback throttled_imu_callback_;

  // computer vision objects
  std::shared_ptr<beam_cv::PoseRefinement> pose_refiner_;
  std::shared_ptr<beam_calibration::CameraModel> cam_model_;
  std::shared_ptr<beam_cv::Tracker> tracker_;
  std::shared_ptr<VisualMap> visual_map_;
  beam_cv::DescriptorType descriptor_type_;
  uint8_t descriptor_type_int_;

  // initialization object
  std::shared_ptr<vision::VIOInitialization> initialization_;

  // imu preintegration object
  std::shared_ptr<ImuPreintegration> imu_preint_;

  // keyframe information
  std::deque<Keyframe> keyframes_;
  uint32_t added_since_kf_{0};

  // robot extrinsics
  Eigen::Matrix4d T_cam_baselink_;
  bs_common::ExtrinsicsLookupOnline &extrinsics_ =
      bs_common::ExtrinsicsLookupOnline::GetInstance();

  // current active submap index
  ActiveSubmap &active_submap_ = ActiveSubmap::GetInstance();
};

} // namespace bs_models
