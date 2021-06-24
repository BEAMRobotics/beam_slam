#pragma once
// libbeam
#include <beam_calibration/CameraModel.h>
#include <beam_cv/geometry/PoseRefinement.h>
#include <beam_cv/geometry/Triangulation.h>
#include <beam_cv/tracker/Tracker.h>
#include <beam_utils/utils.h>
// fuse
#include <beam_common/utils.h>
#include <beam_models/camera_to_camera/initial/imu_initializer.h>
#include <beam_models/camera_to_camera/visual_map.h>
#include <beam_models/frame_to_frame/imu_preintegration.h>
#include <fuse_graphs/hash_graph.h>
// ros
#include <nav_msgs/Path.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

namespace beam_models { namespace camera_to_camera {

class LVIOInitializer {
public:
  LVIOInitializer() = default;

  /**
   * @brief Custom Constructor
   */
  LVIOInitializer(std::shared_ptr<beam_calibration::CameraModel> cam_model,
                  std::shared_ptr<beam_cv::Tracker> tracker,
                  std::shared_ptr<beam_cv::PoseRefinement> pose_refiner,
                  const Eigen::Matrix4d& T_imu_cam)
      : cam_model_(cam_model),
        tracker_(tracker),
        pose_refiner_(pose_refiner),
        T_imu_cam_(T_imu_cam) {
    local_graph_ = std::make_shared<fuse_graphs::HashGraph>();
    visual_map_ =
        std::make_shared<VisualMap>(cam_model_, local_graph_, T_imu_cam);
  }

  /**
   * @brief Adds an image to the initializer, returns pass or fail
   * @param cur_img image to add
   * @param cur_time timestamp of image
   */
  bool AddImage(ros::Time cur_time) {
    frame_times_.push_back(cur_time.toNSec());
    if (path_available_) {
      // build frames vector for imu initialization
      std::vector<beam_models::camera_to_camera::Frame> valid_frames;
      std::vector<beam_models::camera_to_camera::Frame> invalid_frames;
      BuildFrameVectors(valid_frames, invalid_frames);
      // perform imu initialization pipeline
      beam_models::camera_to_camera::IMUInitializer imu_init(valid_frames);
      imu_init.SolveGyroBias();
      imu_init.SolveGravityAndScale();
      imu_init.RefineGravityAndScale();
      imu_init.SolveAccelBias();
      beam_models::frame_to_frame::ImuPreintegration::Params imu_params;
      imu_params.gravity = imu_init.gravity();
      imu_preint_ =
          std::make_shared<beam_models::frame_to_frame::ImuPreintegration>(
              imu_params, imu_init.bg(), imu_init.ba());
      // add initial poses and imu constraints
      for (int i = 0; i < valid_frames.size(); i++) {
        // 1. Add frames pose to graph
        beam_models::camera_to_camera::Frame frame = valid_frames[i];
        Eigen::Matrix4d T_WORLD_IMU;
        beam::QuaternionAndTranslationToTransformMatrix(frame.q, frame.p,
                                                        T_WORLD_IMU);
        Eigen::Matrix4d T_WORLD_CAM = T_WORLD_IMU * T_imu_cam_;
        visual_map_->addPose(T_WORLD_CAM, frame.t);
        // // 2. Push its imu messages
        // for (auto& imu_data : frame.preint.data) {
        //   imu_preint_->AddToBuffer(imu_data);
        // }
        // // 3. Add respective imu constraints
        // if (i == 0) {
        //   imu_preint_->SetStart(frame.t, visual_map_->getOrientation(frame.t),
        //                         visual_map_->getPosition(frame.t));
        // } else {
        //   fuse_variables::Orientation3DStamped::SharedPtr img_orientation =
        //       visual_map_->getOrientation(frame.t);
        //   fuse_variables::Position3DStamped::SharedPtr img_position =
        //       visual_map_->getPosition(frame.t);
        //   auto transaction = imu_preint_
        //                          ->RegisterNewImuPreintegratedFactor(
        //                              frame.t, img_orientation, img_position)
        //                          .GetTransaction();
        //   for (auto& var : transaction->addedVariables()) {
        //     fuse_core::Variable::UniquePtr var_unique = var.clone();
        //     fuse_core::Variable::SharedPtr var_shared = std::move(var_unique);
        //     local_graph_->addVariable(var_shared);
        //   }
        //   for (auto& constraint : transaction->addedConstraints()) {
        //     fuse_core::Constraint::UniquePtr constraint_unique =
        //         constraint.clone();
        //     fuse_core::Constraint::SharedPtr constraint_shared =
        //         std::move(constraint_unique);
        //     local_graph_->addConstraint(constraint_shared);
        //   }
        // }
      }

      ros::Time start = path_.poses[0].header.stamp;
      ros::Time end = path_.poses[path_.poses.size() - 1].header.stamp;
      std::vector<uint64_t> landmarks =
          tracker_->GetLandmarkIDsInWindow(start, end);
      int num_landmarks = 0;
      bool one_lm = false;
      for (auto& id : landmarks) {
        if(one_lm) break;
        if (!visual_map_->getLandmark(id)) {
          std::vector<Eigen::Matrix4d, beam_cv::AlignMat4d> T_cam_world_v;
          std::vector<Eigen::Vector2i, beam_cv::AlignVec2i> pixels;
          std::vector<ros::Time> observation_stamps;
          beam_cv::FeatureTrack track = tracker_->GetTrack(id);
          for (auto& m : track) {
            beam::opt<Eigen::Matrix4d> T = visual_map_->getPose(m.time_point);
            // check if the pose is in the graph (keyframe)
            if (T.has_value()) {
              pixels.push_back(m.value.cast<int>());
              T_cam_world_v.push_back(T.value());
              observation_stamps.push_back(m.time_point);
            }
          }
          if (T_cam_world_v.size() >= 2) {
            beam::opt<Eigen::Vector3d> point =
                beam_cv::Triangulation::TriangulatePoint(cam_model_,
                                                         T_cam_world_v, pixels);
            if (point.has_value()) {
              num_landmarks++;
              visual_map_->addLandmark(point.value(), id);
              for (int i = 0; i < observation_stamps.size(); i++) {
                one_lm = true;
                visual_map_->addConstraint(
                    observation_stamps[i], id,
                    tracker_->Get(observation_stamps[i], id));
              }
            }
          }
        }
      }
      std::vector<double> residuals;
      double cost;
      local_graph_->evaluate(&cost, &residuals);
      for(auto& r: residuals){
        std::cout << r << std::endl;
      }

      // for (auto& f : valid_frames) {
      //   std::cout << visual_map_->getPose(f.t) << std::endl;
      // }
      // std::cout << "Initial map landmarks: " << num_landmarks << std::endl;
      // ceres::Solver::Options options;
      // options.minimizer_progress_to_stdout = true;
      // options.max_num_iterations = 100;
      // options.max_solver_time_in_seconds = 1e6;
      // options.function_tolerance = 1e-20;
      // options.gradient_tolerance = 1e-20;
      // options.parameter_tolerance = 1e-20;
      // options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
      // local_graph_->optimize(options);
      // for (auto& f : valid_frames) {
      //   std::cout << visual_map_->getPose(f.t) << std::endl;
      // }
      is_initialized_ = true;
      return true;
    }
    return false;
  }

  /**
   * @brief Adds an imu measurement to the initializer
   * @param ang_vel angular velocity
   * @param lin_accel linear acceleration
   * @param cur_time timestamp of imu measurement
   */
  void AddIMU(const sensor_msgs::Imu& msg) { imu_buffer_.push(msg); }
  /**
   * @brief Callback for image processing, this callback will add visual
   * constraints and triangulate new landmarks when required
   * @param[in] msg - The image to process
   */
  void SetPath(const nav_msgs::Path& msg) {
    path_ = msg;
    path_available_ = true;
  }

  /**
   * @brief Returns the current state
   */
  bool Initialized() { return is_initialized_; }

  /**
   * @brief Returns a read only the current local graph
   */
  const fuse_graphs::HashGraph& GetGraph() { return *local_graph_; }

  /**
   * @brief Returns a pointer to the imu preintegration object used
   */
  std::shared_ptr<beam_models::frame_to_frame::ImuPreintegration>
      GetPreintegrator() {
    return imu_preint_;
  }

  void BuildFrameVectors(
      std::vector<beam_models::camera_to_camera::Frame>& valid_frames,
      std::vector<beam_models::camera_to_camera::Frame>& invalid_frames) {
    if (!path_available_) {
      ROS_WARN("Attempting to build frame vectors without available path.");
      return;
    }
    valid_frames.clear();
    invalid_frames.clear();
    // get start and end time of input path
    ros::Time start = path_.poses[0].header.stamp;
    ros::Time end = path_.poses[path_.poses.size() - 1].header.stamp;
    for (auto& kf : frame_times_) {
      ros::Time stamp;
      stamp.fromNSec(kf);
      if (stamp < start || stamp > end) continue;
      // add imu data to frames preintegrator
      beam_common::PreIntegrator preintegrator;
      while (imu_buffer_.front().header.stamp <= stamp) {
        beam_common::IMUData imu_data(imu_buffer_.front());
        preintegrator.data.push_back(imu_data);
        imu_buffer_.pop();
      }
      // if (stamp < end) {
      // get pose of frame using path
      Eigen::Matrix4d T_WORLD_IMU;
      beam_common::InterpolateTransformFromPath(path_, stamp, T_WORLD_IMU);
      Eigen::Vector3d p_WORLD_IMU;
      Eigen::Quaterniond q_WORLD_IMU;
      beam::TransformMatrixToQuaternionAndTranslation(T_WORLD_IMU, q_WORLD_IMU,
                                                      p_WORLD_IMU);
      // create frame and add to valid frame vector
      beam_models::camera_to_camera::Frame new_frame(
          stamp, p_WORLD_IMU, q_WORLD_IMU, preintegrator);
      valid_frames.push_back(new_frame);
      //}
      // else {
      //   // get pose of frame using path
      //   Eigen::Matrix4d T_WORLD_IMU = Eigen::Matrix4d::Zero();
      //   Eigen::Vector3d p_WORLD_IMU;
      //   Eigen::Quaterniond q_WORLD_IMU;
      //   beam::TransformMatrixToQuaternionAndTranslation(
      //       T_WORLD_IMU, q_WORLD_IMU, p_WORLD_IMU);
      //   // create frame and add to invalid frame vector
      //   beam_models::camera_to_camera::Frame new_frame(
      //       stamp, p_WORLD_IMU, q_WORLD_IMU, preintegrator);
      //   invalid_frames.push_back(new_frame);
      // }
    }
  }

protected:
  std::shared_ptr<beam_calibration::CameraModel> cam_model_;
  std::shared_ptr<beam_cv::PoseRefinement> pose_refiner_;
  std::shared_ptr<beam_cv::Tracker> tracker_;
  std::shared_ptr<VisualMap> visual_map_;

  std::queue<sensor_msgs::Imu> imu_buffer_;
  std::vector<uint64_t> frame_times_;

  std::shared_ptr<beam_models::frame_to_frame::ImuPreintegration> imu_preint_;
  std::shared_ptr<fuse_graphs::HashGraph> local_graph_;

  nav_msgs::Path path_;
  bool path_available_ = false;
  bool is_initialized_ = false;
  Eigen::Matrix4d T_imu_cam_;
};
}} // namespace beam_models::camera_to_camera
