#pragma once

#include <list>

#include <fuse_core/async_sensor_model.h>
#include <fuse_core/fuse_macros.h>
#include <fuse_core/throttled_callback.h>
#include <fuse_graphs/hash_graph.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <beam_calibration/CameraModel.h>
#include <beam_containers/LandmarkContainer.h>
#include <beam_cv/ImageDatabase.h>
#include <beam_cv/descriptors/Descriptors.h>

#include <bs_common/bs_msgs.h>
#include <bs_common/extrinsics_lookup_online.h>
#include <bs_models/frame_initializers/frame_initializer.h>
#include <bs_models/imu/imu_preintegration.h>
#include <bs_models/lidar/lidar_path_init.h>
#include <bs_models/vision/visual_map.h>
#include <bs_parameters/models/calibration_params.h>
#include <bs_parameters/models/slam_initialization_params.h>

namespace bs_models {

enum class InitMode { VISUAL = 0, LIDAR };

class SLAMInitialization : public fuse_core::AsyncSensorModel {
public:
  FUSE_SMART_PTR_DEFINITIONS(SLAMInitialization);

  /**
   * @brief Default Constructor
   */
  SLAMInitialization();

  /**
   * @brief Default Destructor
   */
  ~SLAMInitialization() override = default;

private:
  /**
   * @brief Callback for image processing, this callback will add visual
   * constraints and triangulate new landmarks when required
   * @param[in] msg - The image to process
   */
  void processCameraMeasurements(
      const bs_common::CameraMeasurementMsg::ConstPtr& msg);

  /**
   * @brief Callback for imu processing, this will make sure the imu messages
   * are added to the buffer at the correct time
   * @param[in] msg - The imu msg to process
   */
  void processIMU(const sensor_msgs::Imu::ConstPtr& msg);

  /**
   * @brief Callback for lidar processing
   * @param[in] msg - The lidar msg to process
   */
  void processLidar(const sensor_msgs::PointCloud2::ConstPtr& msg);

  /**
   * @brief Process using a frame initializer if desired
   * @param[in] timestamp time of the current request
   */
  void processFrameInit(const ros::Time& timestamp);

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
   * @brief Unsubscribe to the input topics and clear memory
   */
  void onStop() override;

  /**
   * @brief Clears all memory and shuts down subscribers
   */
  void shutdown();

  /**
   * @brief Attempts initialization using the available imu, lidar, and visual
   * data. Will create and send an initial graph to the fuse optimizer to
   * bootstrap odometry.
   * @return pass or fail
   */
  bool Initialize();

  /**
   * @brief Triangulates a landmark
   * @return position of landmark if it can be estimated
   */
  beam::opt<Eigen::Vector3d>
      TriangulateLandmark(const uint64_t lm_id,
                          Eigen::Vector3d& average_viewing_angle,
                          uint64_t& visual_word_id);

  /**
   * @brief If the path was estimated using FRAMEINIT or LIDAR, then we
   * interpolate the poses for the visual measurements
   */
  void InterpolateVisualMeasurements();

  /**
   * @brief Scales and aligns the init path and velocities using gravity and
   * scale estimates
   */
  void AlignPathAndVelocities(bool apply_scale);

  /**
   * @brief Adds poses from the init_path to the graph and adds the imu
   * constraints between them
   */
  void AddPosesAndInertialConstraints();

  /**
   * @brief Adds visual constraints to the graph
   */
  void AddVisualConstraints();

  /**
   * @brief Adds lidar constraints to the graph
   */
  void AddLidarConstraints();

  /**
   * @brief Sends the local graph to the fuse optimizer
   */
  void SendInitializationGraph();

  /**
   * @brief Outputs initialization results as a point cloud and as a pose file
   */
  void OutputResults();

  /**
   * @brief Adds visual measurements to the landmark container
   * @param msg
   */
  void AddMeasurementsToContainer(
      const bs_common::CameraMeasurementMsg::ConstPtr& msg);

  fuse_core::UUID device_id_; //!< The UUID of this device

  // calibration parameters
  bs_parameters::models::CalibrationParams calibration_params_;

  // loadable parameters
  bs_parameters::models::SLAMInitializationParams params_;

  // subscribers
  ros::Subscriber visual_measurement_subscriber_;
  ros::Subscriber imu_subscriber_;
  ros::Subscriber lidar_subscriber_;

  // method for estimating initial path
  InitMode mode_ = InitMode::VISUAL;

  // initial path estimate for performing initialization, stored as
  // T_WORLD_BASELINK
  std::map<uint64_t, Eigen::Matrix4d> init_path_;

  // initial imu estimates
  std::map<uint64_t, Eigen::Vector3d> velocities_;
  Eigen::Vector3d gravity_;
  Eigen::Vector3d bg_;
  Eigen::Vector3d ba_;
  double scale_;

  // data storage
  std::list<sensor_msgs::Imu> imu_buffer_;
  std::shared_ptr<beam_containers::LandmarkContainer> landmark_container_;
  std::list<ros::Time> frame_init_buffer_;
  ros::Time prev_frame_{ros::Time(0)};
  double last_lidar_scan_time_s_{0};
  std::shared_ptr<beam_cv::ImageDatabase> image_db_;

  // measurement buffer sizes
  int max_landmark_container_size_;
  int imu_buffer_size_;
  int lidar_buffer_size_;

  // params only tunable here
  double min_lidar_scan_period_s_{0.1};

  // objects for intializing
  std::shared_ptr<beam_calibration::CameraModel> cam_model_;
  Eigen::Matrix3d cam_intrinsic_matrix_;
  cv::Mat K_;
  std::shared_ptr<vision::VisualMap> visual_map_;
  std::shared_ptr<ImuPreintegration> imu_preint_;
  std::unique_ptr<LidarPathInit> lidar_path_init_;
  bs_models::ImuPreintegration::Params imu_params_;
  std::unique_ptr<bs_models::FrameInitializer> frame_initializer_;
  fuse_core::Graph::SharedPtr local_graph_;

  // extrinsics
  Eigen::Matrix4d T_cam_baselink_;
  Eigen::Matrix4d T_lidar_baselink_;
  Eigen::Matrix4d T_imu_baselink_;
  bs_common::ExtrinsicsLookupOnline& extrinsics_ =
      bs_common::ExtrinsicsLookupOnline::GetInstance();

  // save for plotting
  pcl::PointCloud<pcl::PointXYZRGBL> graph_poses_before_opt_;
  std::stringstream graph_before_opt_;
  pcl::PointCloud<pcl::PointXYZRGBL> lidar_constraints_before_opt_;
  pcl::PointCloud<pcl::PointXYZRGBL> imu_constraints_before_opt_;

  // throttled callbacks for messages
  using ThrottledMeasurementCallback =
      fuse_core::ThrottledMessageCallback<bs_common::CameraMeasurementMsg>;
  ThrottledMeasurementCallback throttled_measurement_callback_;
  using ThrottledIMUCallback =
      fuse_core::ThrottledMessageCallback<sensor_msgs::Imu>;
  ThrottledIMUCallback throttled_imu_callback_;
  using ThrottledLidarCallback =
      fuse_core::ThrottledMessageCallback<sensor_msgs::PointCloud2>;
  ThrottledLidarCallback throttled_lidar_callback_;
};

} // namespace bs_models
