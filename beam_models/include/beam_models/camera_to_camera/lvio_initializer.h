#pragma once
// libbeam
#include <beam_calibration/CameraModel.h>
#include <beam_cv/geometry/PoseRefinement.h>
#include <beam_cv/geometry/Triangulation.h>
#include <beam_cv/tracker/Tracker.h>
#include <beam_utils/utils.h>
// fuse
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
                  const Eigen::Matrix4d& T_imu_cam) {
    this->cam_model_ = cam_model;
    this->tracker_ = tracker;
    this->pose_refiner_ = pose_refiner;
    this->local_graph_ = std::make_shared<fuse_graphs::HashGraph>();
    this->visual_map_ =
        std::make_shared<VisualMap>(cam_model_, local_graph_, T_imu_cam);
    this->T_imu_cam_ = T_imu_cam;
    T_body_vicon_ << 0.33638, -0.01749, 0.94156, 0.06901, //
        -0.02078, -0.99972, -0.01114, -0.02781,           //
        0.94150, -0.01582, -0.33665, -0.12395,            //
        0.0, 0.0, 0.0, 1.0;                               //
  }

  /**
   * @brief Adds an image to the initializer, returns pass or fail
   * @param cur_img image to add
   * @param cur_time timestamp of image
   */
  bool AddKeyframe(ros::Time cur_time) {
    keyframes_.push_back(cur_time.toNSec());
    if (path_available_) {
      std::cout << "processing path" << std::endl;
      Eigen::Vector3d bg = Eigen::Vector3d::Zero(),
                      ba = Eigen::Vector3d::Zero();
      beam_models::frame_to_frame::ImuPreintegration::Params imu_params;
      imu_preint_ =
          std::make_shared<beam_models::frame_to_frame::ImuPreintegration>(
              imu_params, bg, ba);
      // 1. get start and end times in path
      std::vector<Eigen::Matrix4d, beam_cv::AlignMat4d> poses;
      ros::Time start, end;
      int i = 0;
      for (int i = 0; i < path_.poses.size(); i++) {
        geometry_msgs::PoseStamped pose = path_.poses[i];
        if (i == 0) { start = pose.header.stamp; }
        if (i == path_.poses.size() - 1) { end = pose.header.stamp; }
        Eigen::Vector3d position;
        position[0] = pose.pose.position.x;
        position[1] = pose.pose.position.y;
        position[2] = pose.pose.position.z;
        Eigen::Quaterniond orientation;
        orientation.w() = pose.pose.orientation.w;
        orientation.x() = pose.pose.orientation.x;
        orientation.y() = pose.pose.orientation.y;
        orientation.z() = pose.pose.orientation.z;
        Eigen::Matrix4d T_world_vicon;
        beam::QuaternionAndTranslationToTransformMatrix(orientation, position,
                                                        T_world_vicon);
        Eigen::Matrix4d T_world_imu = T_body_vicon_ * T_world_vicon;
        poses.push_back(T_world_imu);
      }
      std::cout << "Start: " << start << std::endl;
      std::cout << "End: " << end << std::endl;
      // 2. add pose variables and imu constraints
      std::vector<uint64_t> added_keyframes;
      for (auto& kf : keyframes_) {
        ros::Time stamp;
        stamp.fromNSec(kf);
        if (stamp < start || stamp > end) continue;
        added_keyframes.push_back(kf);
        // get pose of this frame and add to graph
        Eigen::Matrix4d T_world_imu = this->InterpolatePose(poses, stamp);
        this->visual_map_->addPose(T_world_imu, stamp);
        // push imu messages
        while (imu_buffer_.front().header.stamp <= stamp) {
          imu_preint_->PopulateBuffer(imu_buffer_.front());
          imu_buffer_.pop();
        }
        // add imu constraints
        if (added_keyframes.size() == 1) {
          imu_preint_->SetStart(stamp, this->visual_map_->getOrientation(stamp),
                                this->visual_map_->getPosition(stamp));
        } else {
          fuse_variables::Orientation3DStamped::SharedPtr img_orientation =
              this->visual_map_->getOrientation(stamp);
          fuse_variables::Position3DStamped::SharedPtr img_position =
              this->visual_map_->getPosition(stamp);
          beam_constraints::frame_to_frame::ImuState3DStampedTransaction
              imu_trans = imu_preint_->RegisterNewImuPreintegratedFactor(
                  stamp, img_orientation, img_position);
          fuse_core::Transaction::SharedPtr transaction =
              imu_trans.GetTransaction();
          for (auto& var : transaction->addedVariables()) {
            fuse_core::Variable::UniquePtr var_unique = var.clone();
            fuse_core::Variable::SharedPtr var_shared = std::move(var_unique);
            local_graph_->addVariable(var_shared);
          }
          for (auto& constraint : transaction->addedConstraints()) {
            fuse_core::Constraint::UniquePtr constraint_unique =
                constraint.clone();
            fuse_core::Constraint::SharedPtr constraint_shared =
                std::move(constraint_unique);
            local_graph_->addConstraint(constraint_shared);
          }
        }
        // get pose of this frame
        // std::cout << "Image time: " << stamp << std::endl;
        // std::cout << T_world_imu * T_imu_cam_ << std::endl;
      }
      // // 3. triangulate landmarks and add to graph
      // std::vector<uint64_t> landmark_ids =
      //     this->tracker_->GetLandmarkIDsInWindow(start, end);
      // for (auto& id : landmark_ids) {
      //   if (!this->visual_map_->getLandmark(id)) {
      //     std::vector<Eigen::Matrix4d, beam_cv::AlignMat4d> T_cam_world_v;
      //     std::vector<Eigen::Vector2i, beam_cv::AlignVec2i> pixels;
      //     std::vector<ros::Time> observation_stamps;
      //     beam_cv::FeatureTrack track = this->tracker_->GetTrack(id);
      //     for (auto& m : track) {
      //       beam::opt<Eigen::Matrix4d> T =
      //           this->visual_map_->getPose(m.time_point);
      //       // check if the pose is in the graph (keyframe)
      //       if (T.has_value()) {
      //         pixels.push_back(m.value.cast<int>());
      //         T_cam_world_v.push_back(T.value());
      //         observation_stamps.push_back(m.time_point);
      //       }
      //     }
      //     if (T_cam_world_v.size() >= 3) {
      //       beam::opt<Eigen::Vector3d> point =
      //           beam_cv::Triangulation::TriangulatePoint(this->cam_model_,
      //                                                    T_cam_world_v,
      //                                                    pixels);
      //       if (point.has_value()) {
      //         this->visual_map_->addLandmark(point.value(), id);
      //         for (int i = 0; i < observation_stamps.size(); i++) {
      //           this->visual_map_->addConstraint(
      //               observation_stamps[i], id,
      //               this->tracker_->Get(observation_stamps[i], id));
      //         }
      //       }
      //     }
      //   }
      // }

      // this->TriangulateLandmarks(
      //     this->tracker_->GetLandmarkIDsInWindow(start, end));

      // // 4. solve poses for keyframes outside of lidar path window
      // ros::Time last_kf_stamp;
      // for (auto& kf : keyframes_) {
      //   last_kf_stamp.fromNSec(kf);
      //   if (last_kf_stamp < end) continue;
      //   // push imu messages to preintegrator
      //   while (imu_buffer_.front().header.stamp <= last_kf_stamp) {
      //     sensor_msgs::Imu imu_msg = imu_buffer_.front();
      //     imu_preint_->PopulateBuffer(imu_msg);
      //     imu_buffer_.pop();
      //   }

      //     Eigen::Matrix4d T_world_frame =
      //         imu_preint_->GetPose(last_kf_stamp) * T_imu_cam_;
      //     // get point correspondences and run pose refinement
      //     std::vector<uint64_t> ids =
      //         this->tracker_->GetLandmarkIDsInImage(last_kf_stamp);
      //     std::vector<Eigen::Vector2i, beam_cv::AlignVec2i> pixels;
      //     std::vector<Eigen::Vector3d, beam_cv::AlignVec3d> points;
      //     for (auto& id : ids) {
      //       Eigen::Vector2i pixel = tracker_->Get(last_kf_stamp,
      //       id).cast<int>(); fuse_variables::Position3D::SharedPtr lm =
      //       this->GetLandmark(id); if (lm) {
      //         Eigen::Vector3d point(lm->data());
      //         pixels.push_back(pixel);
      //         points.push_back(point);
      //       }
      //     }
      //     // determine pose and add to graph
      //     Eigen::Matrix4d T_world_frame =
      //         imu_preint_->GetPose(last_kf_stamp) * T_imu_cam_;
      //     T_world_frame = pose_refiner_->RefinePose(T_world_frame,
      //     cam_model_,
      //                                               pixels, points);
      //     this->visual_map_->addPose(T_world_frame, kf);
      //     // add constraints to the seen landmarks
      //     for (auto& id : ids) {
      //       Eigen::Vector2i pixel = tracker_->Get(last_kf_stamp,
      //       id).cast<int>(); fuse_variables::Position3D::SharedPtr lm =
      //           this->visual_map_->getLandmark(id);
      //       if (lm) {
      //         this->visual_map_->addConstraint(last_kf_stamp, id, pixel);
      //       }
      //     }
      //     // 5. triangulate more landmarks and add to graph
      //     this->TriangulateLandmarks(
      //         this->tracker_->GetLandmarkIDsInWindow(end, last_kf_stamp));
      //   }

      //   // 6. optimize graph
      //   local_graph_->optimize();
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

  fuse_core::Graph::const_variable_range GetVariables();

  fuse_core::Graph::const_constraint_range GetConstraints();

  std::shared_ptr<beam_models::frame_to_frame::ImuPreintegration>
      GetPreintegrator() {
    return imu_preint_;
  }

private:
  Eigen::Matrix4d
      InterpolatePose(std::vector<Eigen::Matrix4d, beam_cv::AlignMat4d> poses,
                      ros::Time time) {
    for (int i = 0; i < path_.poses.size(); i++) {
      if (time < path_.poses[i + 1].header.stamp &&
          time >= path_.poses[i].header.stamp) {
        Eigen::Matrix4d pose1 = poses[i];
        Eigen::Matrix4d pose2 = poses[i + 1];
        Eigen::Matrix4d interpolated = beam::InterpolateTransform(
            pose1, beam::RosTimeToChrono(path_.poses[i].header.stamp), pose2,
            beam::RosTimeToChrono(path_.poses[i + 1].header.stamp),
            beam::RosTimeToChrono(time));
        return interpolated;
      }
    }
  }

  void TriangulateLandmarks(std::vector<uint64_t> landmark_ids) {
    for (auto& id : landmark_ids) {
      if (!this->visual_map_->getLandmark(id)) {
        std::vector<Eigen::Matrix4d, beam_cv::AlignMat4d> T_cam_world_v;
        std::vector<Eigen::Vector2i, beam_cv::AlignVec2i> pixels;
        std::vector<ros::Time> observation_stamps;
        beam_cv::FeatureTrack track = this->tracker_->GetTrack(id);
        for (auto& m : track) {
          beam::opt<Eigen::Matrix4d> T =
              this->visual_map_->getPose(m.time_point);
          // check if the pose is in the graph (keyframe)
          if (T.has_value()) {
            pixels.push_back(m.value.cast<int>());
            T_cam_world_v.push_back(T.value());
            observation_stamps.push_back(m.time_point);
          }
        }
        if (T_cam_world_v.size() >= 3) {
          beam::opt<Eigen::Vector3d> point =
              beam_cv::Triangulation::TriangulatePoint(this->cam_model_,
                                                       T_cam_world_v, pixels);
          if (point.has_value()) {
            this->visual_map_->addLandmark(point.value(), id);
            for (int i = 0; i < observation_stamps.size(); i++) {
              this->visual_map_->addConstraint(
                  observation_stamps[i], id,
                  this->tracker_->Get(observation_stamps[i], id));
            }
          }
        }
      }
    }
  }

protected:
  std::shared_ptr<beam_calibration::CameraModel> cam_model_;
  std::shared_ptr<beam_cv::PoseRefinement> pose_refiner_;
  std::shared_ptr<beam_cv::Tracker> tracker_;
  std::shared_ptr<VisualMap> visual_map_;

  std::queue<sensor_msgs::Imu> imu_buffer_;
  std::vector<uint64_t> keyframes_;

  std::shared_ptr<beam_models::frame_to_frame::ImuPreintegration> imu_preint_;
  std::shared_ptr<fuse_graphs::HashGraph> local_graph_;

  nav_msgs::Path path_;
  bool path_available_ = false;
  bool is_initialized_ = false;
  Eigen::Matrix4d T_body_vicon_, T_imu_cam_;
};

}} // namespace beam_models::camera_to_camera
