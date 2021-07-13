#pragma once
// libbeam
#include <beam_calibration/CameraModel.h>
#include <beam_cv/geometry/Triangulation.h>
#include <beam_cv/trackers/Trackers.h>
#include <beam_utils/utils.h>
// fuse
#include <beam_common/preintegrator.h>
#include <beam_common/utils.h>
#include <beam_models/camera_to_camera/visual_map.h>
#include <beam_models/frame_to_frame/imu_preintegration.h>
#include <beam_models/trajectory_initializers/imu_initializer.h>
#include <fuse_graphs/hash_graph.h>
// ros
#include <beam_models/InitializedPathMsg.h>
#include <sensor_msgs/Imu.h>

namespace beam_models { namespace camera_to_camera {

class VIOInitializer {
public:
  /**
   * @brief Default Constructor
   */
  VIOInitializer() = default;

  /**
   * @brief Custom Constructor
   */
  VIOInitializer(std::shared_ptr<beam_calibration::CameraModel> cam_model,
                 std::shared_ptr<beam_cv::Tracker> tracker,
                 const double& gyro_noise, const double& accel_noise,
                 const double& gyro_bias, const double& accel_bias,
                 bool use_scale_estimate = false);

  /**
   * @brief Notifies initialization that image at this time is to be used for
   * init
   * @param cur_time timestamp of image
   */
  bool AddImage(ros::Time cur_time);

  /**
   * @brief Adds an imu measurement to the initializer
   * @param msg imu message to add
   */
  void AddIMU(const sensor_msgs::Imu& msg);

  /**
   * @brief Sets the initialized path to use
   * @param msg The init path to use for intialization
   */
  void SetPath(const InitializedPathMsg& msg);

  /**
   * @brief Returns the current state
   * @return true or false if process is initialized
   */
  bool Initialized();

  /**
   * @brief Returns a read only the current local graph
   * @return read-only graph
   */
  const fuse_graphs::HashGraph& GetGraph();

  /**
   * @brief Returns a pointer to the imu preintegration object used
   * @return point to imu preintegration
   */
  std::shared_ptr<beam_models::frame_to_frame::ImuPreintegration>
      GetPreintegrator();

private:
  /**
   * @brief Build a vector of frames with the current init path and the
   * frame_times_ vector, if a frame is outside of the given path it will be
   * referred to as an invalid frame and its pose will be set to 0
   * @param valid_frames[out] frames that are within the init path
   * @param invalid_frames[out] frames that are outside the init path
   */
  void BuildFrameVectors(
      std::vector<beam_models::camera_to_camera::Frame>& valid_frames,
      std::vector<beam_models::camera_to_camera::Frame>& invalid_frames);

  /**
   * @brief Estimates imu parameters given a vector of frames with some known
   * poses (can be up to scale from sfm, or in real world scale from lidar)
   * @param frames input frames  with poses to estimate imu parameters
   */
  void PerformIMUInitialization(
      const std::vector<beam_models::camera_to_camera::Frame>& frames);

  /**
   * @brief Adds all poses and inertial constraints contained within the frames
   * vector to the local graph
   * @param frames input frames
   * @param set_start when true will set the first frames pose as the prior
   */
  void AddPosesAndInertialConstraints(
      const std::vector<beam_models::camera_to_camera::Frame>& frames,
      bool set_start);

  /**
   * @brief Adds visual constraints to input frames, will triangulate landmarks
   * if they are not already triangulated
   * @param frames input frames
   * @return number of landmarks that have been added
   */
  size_t AddVisualConstraints(
      const std::vector<beam_models::camera_to_camera::Frame>& frames);

  /**
   * @brief Localizes a given frame using the current landmarks
   * @param frames input frames
   * @param T_WORLD_CAMERA[out] estimated pose of the camera
   * @return true or false if it succeeded or not
   */
  bool LocalizeFrame(const beam_models::camera_to_camera::Frame& frame,
                     Eigen::Matrix4d& T_WORLD_CAMERA);

  /**
   * @brief Optimizes the current local graph
   */
  void OptimizeGraph();

protected:
  // computer vision objects
  std::shared_ptr<beam_calibration::CameraModel> cam_model_;
  std::shared_ptr<beam_cv::Tracker> tracker_;
  std::shared_ptr<beam_models::camera_to_camera::VisualMap> visual_map_;

  // imu preintegration object
  std::shared_ptr<beam_models::frame_to_frame::ImuPreintegration> imu_preint_;

  // graph object for optimization
  std::shared_ptr<fuse_graphs::HashGraph> local_graph_;

  // stores the added imu messages and times of keyframes to use for init
  std::queue<sensor_msgs::Imu> imu_buffer_;
  std::vector<uint64_t> frame_times_;

  // boolean flags
  bool is_initialized_ = false;
  bool use_scale_estimate_ = false;

  // imu intrinsics
  Eigen::Matrix3d cov_gyro_noise_;
  Eigen::Matrix3d cov_accel_noise_;
  Eigen::Matrix3d cov_gyro_bias_;
  Eigen::Matrix3d cov_accel_bias_;

  // preintegration parameters
  Eigen::Vector3d gravity_, bg_, ba_;
  double scale_;

  // initialization path
  std::shared_ptr<InitializedPathMsg> init_path_;
};

}} // namespace beam_models::camera_to_camera
