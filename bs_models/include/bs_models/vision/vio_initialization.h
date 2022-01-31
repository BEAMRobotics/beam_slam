#pragma once

#include <fuse_graphs/hash_graph.h>
#include <sensor_msgs/Imu.h>

#include <beam_calibration/CameraModel.h>
#include <beam_cv/geometry/PoseRefinement.h>
#include <beam_cv/trackers/Trackers.h>

#include <bs_common/bs_msgs.h>
#include <bs_common/extrinsics_lookup_online.h>
#include <bs_models/global_mapping/active_submap.h>
#include <bs_models/imu/imu_preintegration.h>
#include <bs_models/vision/visual_map.h>

using namespace bs_common;

namespace bs_models { namespace vision {

struct Frame {
  ros::Time t;
  Eigen::Vector3d p;
  Eigen::Quaterniond q;
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
                    const std::string& imu_intrinsics_path,
                    bool use_scale_estimate = false,
                    double max_optimization_time = 1.0,
                    const std::string& output_directory = "");

  /**
   * @brief Attempt initialization at image at given time stamp
   * @param cur_time timestamp of image
   * @return success flag
   */
  bool AddImage(const ros::Time& cur_time);

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
   * @brief Returns a read only version of the current local graph
   * @return read-only graph
   */
  const fuse_core::Graph::SharedPtr& GetGraph();

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
   * @brief Build a vector of valid frames (that fall within time window of
   * given init path) and invalid frames (that fall after the window of given
   * init path)
   */
  void BuildFrameVectors();

  /**
   * @brief Adds all poses of given frames to the graph and te inertial
   * constraints between them
   * @param frames input frames
   * @param is_valid flag to notify if the frames are valid (have poses and velocities)
   */
  void AddPosesAndInertialConstraints(const std::vector<Frame>& frames,
                                      bool is_valid);

  /**
   * @brief Adds visual constraints to input frames, will triangulate landmarks
   * if they are not already triangulated
   * @param frames input frames
   * @return number of landmarks that have been added
   */
  size_t AddVisualConstraints(const std::vector<Frame>& frames);

  /**
   * @brief Localizes a given frame using the tracker and the current visual map
   * @param img_time time of image to localize
   * @return T_WORLD_BASELINK
   */
  Eigen::Matrix4d LocalizeFrame(const ros::Time& img_time);

  /**
   * @brief Optimizes the current local graph
   */
  void OptimizeGraph();

  /**
   * @brief Aligns the valid frames and init path to estimated gravity
   */
  void AlignPosesToGravity();

  /**
   * @brief Saves the poses and the points from the given frames to point clouds
   */
  void OutputResults();

protected:
  // subscriber for initialized path
  ros::Subscriber path_subscriber_;

  // computer vision objects
  std::shared_ptr<beam_calibration::CameraModel> cam_model_;
  std::shared_ptr<beam_cv::Tracker> tracker_;
  std::shared_ptr<bs_models::vision::VisualMap> visual_map_;
  std::shared_ptr<beam_cv::PoseRefinement> pose_refiner_;

  // imu preintegration object
  std::shared_ptr<bs_models::ImuPreintegration> imu_preint_;
  bs_models::ImuPreintegration::Params imu_params_;

  // graph object for optimization
  fuse_core::Graph::SharedPtr local_graph_;
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

  // preintegration parameters
  Eigen::Vector3d gravity_{0, 0, 0};
  Eigen::Vector3d bg_{0, 0, 0};
  Eigen::Vector3d ba_{0, 0, 0};
  std::vector<Eigen::Vector3d> velocities_;
  double scale_{1};

  // extrinsic info
  Eigen::Matrix4d T_cam_baselink_;
  bs_common::ExtrinsicsLookupOnline& extrinsics_ =
      bs_common::ExtrinsicsLookupOnline::GetInstance();

  // initialization path
  std::shared_ptr<InitializedPathMsg> init_path_;

  // directory to optionally output the initialization results
  std::string output_directory_;
};
}} // namespace bs_models::vision
