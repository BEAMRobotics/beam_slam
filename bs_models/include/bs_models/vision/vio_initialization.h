#pragma once

#include <fuse_graphs/hash_graph.h>
#include <sensor_msgs/Imu.h>

#include <beam_calibration/CameraModel.h>
#include <beam_cv/geometry/PoseRefinement.h>
#include <beam_cv/trackers/Trackers.h>

#include <bs_common/bs_msgs.h>
#include <bs_common/extrinsics_lookup_online.h>
#include <bs_models/current_submap.h>
#include <bs_models/imu_preintegration.h>
#include <bs_models/vision/visual_map.h>

using namespace bs_common;

namespace bs_models { namespace vision {

struct Frame {
  ros::Time t;
  Eigen::Vector3d p;
  Eigen::Quaterniond q;
  bs_common::PreIntegrator preint;
};

class VIOInitialization {
public:
  /**
   * @brief Default Constructor
   */
  VIOInitialization() = default;

  /**
   * @brief Custom Constructor
   */
  VIOInitialization(std::shared_ptr<beam_calibration::CameraModel> cam_model,
                 std::shared_ptr<beam_cv::Tracker> tracker,
                 const std::string& path_topic,
                 const std::string& imu_intrinsics_path,
                 bool use_scale_estimate = false,
                 double max_optimization_time = 5.0,
                 const std::string& output_directory = "");

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
  std::shared_ptr<bs_models::ImuPreintegration> GetPreintegrator();

  /**
   * @brief Callback for path processing, this path is provided by LIO for
   * initialization
   * @param[in] msg - The path to process
   */
  void ProcessInitPath(const InitializedPathMsg::ConstPtr& msg);

private:
  /**
   * @brief Build a vector of frames with the current init path and the
   * frame_times_ vector, if a frame is outside of the given path it will be
   * referred to as an invalid frame and its pose will be set to 0
   */
  void BuildFrameVectors();

  /**
   * @brief Estimates imu parameters given a vector of frames with some known
   * poses (can be up to scale from sfm, or in real world scale from lidar)
   * @param frames input frames with poses to estimate imu parameters
   */
  void PerformIMUInitialization(std::vector<Frame>& frames);

  /**
   * @brief Adds all poses and inertial constraints contained within the frames
   * vector to the local graph
   * @param frames input frames
   * @param set_start when true will set the first frames pose as the prior
   */
  void AddPosesAndInertialConstraints(const std::vector<Frame>& frames,
                                      bool set_start);

  /**
   * @brief Adds visual constraints to input frames, will triangulate landmarks
   * if they are not already triangulated
   * @param frames input frames
   * @return number of landmarks that have been added
   */
  size_t AddVisualConstraints(const std::vector<Frame>& frames);

  /**
   * @brief Localizes a given frame using the current landmarks
   * @param frames input frames
   * @param T_WORLD_BASELINK[out] estimated pose of the camera
   * @return true or false if it succeeded or not
   */
  bool LocalizeFrame(const Frame& frame, Eigen::Matrix4d& T_WORLD_BASELINK);

  /**
   * @brief Outputs frame poses to standard output
   * @param frames vector of frames to output
   */
  void OutputFramePoses(const std::vector<Frame>& frames);

  /**
   * @brief Optimizes the current local graph
   */
  void OptimizeGraph();

  /**
   * @brief Saves the poses and the points from the given frames to point clouds
   * @param frames input frames
   */
  void OutputResults(const std::vector<Frame>& frames);

  /**
   * @brief Solves for the gyroscope bias
   */
  void SolveGyroBias(std::vector<Frame>& frames);

  /**
   * @brief Solves for acceleromater bias (gravity must be estimated before
   * calling this)
   */
  void SolveAccelBias(std::vector<Frame>& frames);

  /**
   * @brief Solves for Gravity and Scale factor (scale factor only used if
   * frames are in arbitrary scale)
   */
  void SolveGravityAndScale(std::vector<Frame>& frames);

  /**
   * @brief Refines the previously estimated gravity and scale factor
   */
  void RefineGravityAndScale(std::vector<Frame>& frames);

  /**
   * @brief Integrates each frame using current bias estimates
   */
  void Integrate(std::vector<Frame>& frames);

  Eigen::Matrix<double, 3, 2> s2_tangential_basis(const Eigen::Vector3d& x) {
    int d = 0;
    for (int i = 1; i < 3; ++i) {
      if (std::abs(x[i]) > std::abs(x[d])) d = i;
    }
    Eigen::Vector3d b1 =
        x.cross(Eigen::Vector3d::Unit((d + 1) % 3)).normalized();
    Eigen::Vector3d b2 = x.cross(b1).normalized();
    return (Eigen::Matrix<double, 3, 2>() << b1, b2).finished();
  }

protected:
  // subscriber for initialized path
  ros::Subscriber path_subscriber_;

  // computer vision objects
  std::shared_ptr<beam_cv::PoseRefinement> pose_refiner_;
  std::shared_ptr<beam_calibration::CameraModel> cam_model_;
  std::shared_ptr<beam_cv::Tracker> tracker_;
  std::shared_ptr<bs_models::vision::VisualMap> visual_map_;
  CurrentSubmap& submap_ = CurrentSubmap::GetInstance();

  // imu preintegration object
  std::shared_ptr<bs_models::ImuPreintegration> imu_preint_;
  bs_models::ImuPreintegration::Params imu_params_;

  // graph object for optimization
  std::shared_ptr<fuse_graphs::HashGraph> local_graph_;
  double max_optimization_time_;

  // stores the added imu messages and times of keyframes to use for init
  std::queue<sensor_msgs::Imu> imu_buffer_;
  std::vector<uint64_t> frame_times_;

  // keyframe information
  std::vector<Frame> valid_frames_;
  std::vector<Frame> invalid_frames_;
  
  // boolean flags
  bool is_initialized_{false};
  bool use_scale_estimate_{false};

  // imu intrinsics
  Eigen::Matrix3d cov_gyro_noise_;
  Eigen::Matrix3d cov_accel_noise_;
  Eigen::Matrix3d cov_gyro_bias_;
  Eigen::Matrix3d cov_accel_bias_;

  // preintegration parameters
  Eigen::Vector3d gravity_;
  Eigen::Vector3d bg_;
  Eigen::Vector3d ba_;
  double scale_;

  // initialization path
  std::shared_ptr<InitializedPathMsg> init_path_;

  // robot extrinsics
  Eigen::Matrix4d T_cam_baselink_;
  ExtrinsicsLookupOnline& extrinsics_ = ExtrinsicsLookupOnline::GetInstance();

  // directory to optionally output the initialization results
  std::string output_directory_;
};

}} // namespace bs_models::trajectory_initializers
