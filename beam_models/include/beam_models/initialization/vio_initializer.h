#pragma once
// libbeam
#include <beam_calibration/CameraModel.h>
#include <beam_cv/geometry/PoseRefinement.h>
#include <beam_cv/geometry/Triangulation.h>
#include <beam_cv/trackers/Trackers.h>
#include <beam_utils/utils.h>
// fuse
#include <beam_common/utils.h>
#include <beam_models/camera_to_camera/visual_map.h>
#include <beam_models/frame_to_frame/imu_preintegration.h>
#include <beam_models/initialization/imu_initializer.h>
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
                 std::shared_ptr<beam_cv::PoseRefinement> pose_refiner,
                 const Eigen::Matrix4d& T_imu_cam,
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
   */
  bool Initialized();

  /**
   * @brief Returns a read only the current local graph
   */
  const fuse_graphs::HashGraph& GetGraph();

  /**
   * @brief Returns a pointer to the imu preintegration object used
   */
  std::shared_ptr<beam_models::frame_to_frame::ImuPreintegration>
      GetPreintegrator();

private:
  /**
   * @brief Returns a pointer to the imu preintegration object used
   */
  void BuildFrameVectors(
      std::vector<beam_models::camera_to_camera::Frame>& valid_frames,
      std::vector<beam_models::camera_to_camera::Frame>& invalid_frames);

  void PerformIMUInitialization(
      const std::vector<beam_models::camera_to_camera::Frame>& frames);

  void AddPosesAndIMUConstraints(
      const std::vector<beam_models::camera_to_camera::Frame>& frames);

  void AddLandmarksAndVisualConstraints();

protected:
  std::shared_ptr<beam_calibration::CameraModel> cam_model_;
  std::shared_ptr<beam_cv::PoseRefinement> pose_refiner_;
  std::shared_ptr<beam_cv::Tracker> tracker_;
  std::shared_ptr<beam_models::camera_to_camera::VisualMap> visual_map_;
  std::shared_ptr<beam_models::frame_to_frame::ImuPreintegration> imu_preint_;
  std::shared_ptr<fuse_graphs::HashGraph> local_graph_;

  std::queue<sensor_msgs::Imu> imu_buffer_;
  std::vector<uint64_t> frame_times_;

  bool is_initialized_ = false;
  bool use_scale_estimate_ = false;
  Eigen::Matrix4d T_imu_cam_;

  std::shared_ptr<InitializedPathMsg> init_path_;
  ros::Time start_, end_;
  Eigen::Vector3d gravity_, bg_, ba_;
  double scale_;
};
}} // namespace beam_models::camera_to_camera
