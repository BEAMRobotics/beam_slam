#pragma once

// std
#include <queue>

// messages
#include <bs_models/CameraMeasurementMsg.h>
#include <bs_models/InitializedPathMsg.h>
#include <bs_models/LandmarkMeasurementMsg.h>
#include <bs_models/SubmapMsg.h>
#include <bs_models/TrajectoryMeasurementMsg.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

// fuse
#include <fuse_core/async_sensor_model.h>
#include <fuse_models/common/throttled_callback.h>

// beam_slam
#include <bs_common/extrinsics_lookup_online.h>
#include <bs_models/camera_to_camera/keyframe.h>
#include <bs_models/camera_to_camera/visual_map.h>
#include <bs_models/camera_to_camera/visual_submap.h>
#include <bs_models/frame_to_frame/imu_preintegration.h>
#include <bs_models/trajectory_initializers/vio_initializer.h>
#include <bs_parameters/models/camera_params.h>
#include <bs_parameters/models/global_params.h>

// libbeam
#include <beam_calibration/CameraModel.h>
#include <beam_cv/geometry/PoseRefinement.h>
#include <beam_cv/trackers/Trackers.h>

namespace bs_models { namespace camera_to_camera {

class VisualInertialOdom : public fuse_core::AsyncSensorModel {
public:
  SMART_PTR_DEFINITIONS(VisualInertialOdom);

  /**
   * @brief Default Constructor
   */
  VisualInertialOdom();

  /**
   * @brief Default Destructor
   */
  ~VisualInertialOdom() override = default;

  /**
   * @brief Callback for image processing, this callback will add visual
   * constraints and triangulate new landmarks when required
   * @param[in] msg - The image to process
   */
  void processImage(const sensor_msgs::Image::ConstPtr& msg);

  /**
   * @brief Callback for imu processing, this will make sure the imu messages
   * are added to the buffer at the correct time
   * @param[in] msg - The imu msg to process
   */
  void processIMU(const sensor_msgs::Imu::ConstPtr& msg);

  /**
   * @brief Callback for path processing, this path is provided by LIO for
   * initialization
   * @param[in] msg - The path to process
   */
  void processInitPath(const InitializedPathMsg::ConstPtr& msg);

  /**
   * @brief Callback for global submap processing
   * @param[in] msg - The submap to process
   */
  void processSubmap(const SubmapMsg::ConstPtr& msg);

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
   * @brief Subscribe to the input topic to start sending transactions to the
   * optimizer
   */
  void onStart() override;

  /**
   * @brief Unsubscribe to the input topic
   */
  void onStop() override;

  /**
   * @brief Callback for when a newly optimized graph is available
   */
  void onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph_msg) override;

private:
  /**
   * @brief Copies all variables and constraints in the init graph and sends to
   * fuse optimizer
   * @param init_graph the graph obtained from the initializer
   */
  void SendInitializationGraph(const fuse_graphs::HashGraph& init_graph);

  /**
   * @brief Localizes a given frame using the tracker and the current visual map
   * @param img_time time of image to localize
   * @param[out] triangulated_ids id's of landmarks that have already been
   * triangulated
   * @param[out] untriangulated_ids id's of landmarks that have not been
   * triangulated
   * @param[out] T_WORLD_CAMERA estimated pose
   * @return failure or success
   */
  bool LocalizeFrame(const ros::Time& img_time,
                     std::vector<uint64_t>& triangulated_ids,
                     std::vector<uint64_t>& untriangulated_ids,
                     Eigen::Matrix4d& T_WORLD_CAMERA);

  /**
   * @brief Determines if a frame is a keyframe
   * @param img_time time of image to determine if its a keyframe
   * @param triangulated_ids id's of landmarks that have already been
   * triangulated
   * @param untriangulated_ids id's of landmarks that have not been
   * triangulated
   * @param T_WORLD_CAMERA pose of the frame in question
   * @return true or false decision
   */
  bool IsKeyframe(const ros::Time& img_time,
                  const std::vector<uint64_t>& triangulated_ids,
                  const std::vector<uint64_t>& untriangulated_ids,
                  const Eigen::Matrix4d& T_WORLD_CAMERA);

  /**
   * @brief Extends the map at an image time and adds the visual constraints
   * @param img_time time of keyframe to extend map at
   * @param triangulated_ids id's of landmarks that have already been
   * triangulated
   * @param untriangulated_ids id's of landmarks that have not been
   * triangulated
   */
  void ExtendMap(const std::vector<uint64_t>& triangulated_ids,
                 const std::vector<uint64_t>& untriangulated_ids);

  /**
   * @brief creates inertial cosntraint for the current keyframe and merges with
   * the input transaction
   * @param transaction current transaction to merge inertial constraints with
   * (typically built from extend map)
   */
  void AddInertialConstraint(fuse_core::Transaction::SharedPtr transaction);

  /**
   * @brief Adds the keyframe pose to the graph and publishes keyframe header to
   * notify any lsiteners
   * @param T_WORLD_CAMERA pose of the keyframe to add
   */
  void NotifyNewKeyframe(const Eigen::Matrix4d& T_WORLD_CAMERA);

  /**
   * @brief Publishes the oldest keyframe that is stored as a slam chunk message
   */
  void PublishSlamChunk();

  /**
   * @brief Publishes landmark ids
   */
  void PublishLandmarkIDs(const std::vector<uint64_t>& ids);

  /**
   * @brief Computes the mean parallax between images at two times
   * @param t1 time of first image
   * @param t2 time of second image
   * @param t2_landmarks id's of landmarks that have been seen in t2
   * @return mean parallax
   */
  double ComputeAvgParallax(const ros::Time& t1, const ros::Time& t2,
                            const std::vector<uint64_t>& t2_landmarks);

  /**
   * @brief Matches current untriangulated id's to the current submap if its
   * available
   * @param untriangulated_landmarks id's of landmarks that have been seen in
   * the current keyframe
   * @return 1:1 vector of vector3d's to untriangulated id's
   */
  std::vector<beam::opt<Eigen::Vector3d>>
      MatchKeyframeToSubmap(const std::vector<uint64_t>& untriangulated_ids);

protected:
  // loadable camera parameters
  bs_parameters::models::CameraParams camera_params_;

  // global parameters
  bs_parameters::models::GlobalParams global_params_;

  // subscribers
  ros::Subscriber image_subscriber_;
  ros::Subscriber imu_subscriber_;
  ros::Subscriber path_subscriber_;
  ros::Subscriber submap_subscriber_;

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
      fuse_models::common::ThrottledCallback<sensor_msgs::Image>;
  using ThrottledIMUCallback =
      fuse_models::common::ThrottledCallback<sensor_msgs::Imu>;
  ThrottledImageCallback throttled_image_callback_;
  ThrottledIMUCallback throttled_imu_callback_;

  // computer vision objects
  std::shared_ptr<beam_cv::PoseRefinement> pose_refiner_;
  std::shared_ptr<beam_calibration::CameraModel> cam_model_;
  std::shared_ptr<beam_cv::Tracker> tracker_;
  std::shared_ptr<bs_models::camera_to_camera::VisualMap> visual_map_;
  std::shared_ptr<bs_models::camera_to_camera::VisualSubmap> current_submap_;

  // initialization object
  std::shared_ptr<bs_models::camera_to_camera::VIOInitializer> initializer_;

  // imu preintegration object
  std::shared_ptr<bs_models::frame_to_frame::ImuPreintegration> imu_preint_;

  // keyframe information
  std::deque<bs_models::camera_to_camera::Keyframe> keyframes_;
  uint32_t added_since_kf_{0};

  // robot extrinsics
  Eigen::Matrix4d T_cam_baselink_;
  bs_common::ExtrinsicsLookupOnline& extrinsics_ =
      bs_common::ExtrinsicsLookupOnline::GetInstance();
};

}} // namespace bs_models::camera_to_camera
