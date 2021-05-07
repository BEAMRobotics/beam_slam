#pragma once
// libbeam
#include <beam_calibration/CameraModels.h>
#include <beam_cv/descriptors/Descriptors.h>
#include <beam_cv/detectors/Detectors.h>
#include <beam_cv/geometry/Triangulation.h>
#include <beam_cv/matchers/Matchers.h>
#include <beam_cv/tracker/Tracker.h>
// beam_slam
#include <beam_constraints/camera_to_camera/visual_constraint.h>
#include <beam_variables/position_3d.h>
// fuse
#include <fuse_core/graph.h>
#include <fuse_core/transaction.h>
#include <fuse_core/uuid.h>

namespace beam_models { namespace camera_to_camera {

typedef beam_containers::LandmarkContainer<
    beam_containers::LandmarkMeasurement<int>>
    LMContainer;

class Camera {
public:
  Camera() = default;

  Camera(std::shared_ptr<beam_cv::Matcher> matcher,
         std::shared_ptr<beam_cv::Descriptor> descriptor,
         std::shared_ptr<beam_cv::Detector> detector,
         std::shared_ptr<beam_calibration::CameraModel> model, int window_size)
      : cam_model_(model) {
    tracker_ = std::make_shared<beam_cv::Tracker>(detector, descriptor, matcher,
                                                  window_size);
  }

  /**
   * @brief Adds the image to the tracker, and returns whether it is selected as
   * a keyframe
   * @param img image message to add
   * @param T_WORLD_SENSOR pose of the associated image
   */
  bool AddImage(cv::Mat cur_img, ros::Time cur_time,
                Eigen::Matrix4d& T_WORLD_SENSOR,
                fuse_core::Transaction::SharedPtr transaction) {
    tracker_->AddImage(cur_img, cur_time);
    img_num_++;

    fuse_variables::Orientation3DStamped::SharedPtr cur_orientation =
        fuse_variables::Orientation3DStamped::make_shared(cur_time);
    fuse_variables::Position3DStamped::SharedPtr cur_position =
        fuse_variables::Position3DStamped::make_shared(cur_time);
    this->SetPoseVariables(T_WORLD_SENSOR, cur_orientation, cur_position);

    if (img_num_ == 1) {
      cur_kf_orientation_ = cur_orientation;
      cur_kf_position_ = cur_position;

      transaction->stamp(cur_time);
      transaction->addVariable(cur_orientation);
      transaction->addVariable(cur_position);
      tracker_orientations_[cur_time.toSec()] = cur_orientation;
      tracker_positions_[cur_time.toSec()] = cur_position;

    } else if (this->IsKeyframe(cur_time, cur_position)) {
      // stamp the transaction and add the pose variables
      transaction->stamp(cur_time);
      transaction->addVariable(cur_orientation);
      transaction->addVariable(cur_position);
      tracker_orientations_[cur_time.toSec()] = cur_orientation;
      tracker_positions_[cur_time.toSec()] = cur_position;
      // look for new landmarks and add visual constraints
      // this->RegisterKeyframe(transaction);
      this->AddNewLandmarks(transaction);
      this->AddExistingFeatureConstraints(transaction);
      // set as new keyframe
      cur_kf_orientation_ = cur_orientation;
      cur_kf_position_ = cur_position;
      return true;
    }
    return false;
  }

  void SetGraph(fuse_core::Graph::ConstSharedPtr graph) {
    this->graph_ = std::move(graph);
    if (!graph_initialized) graph_initialized = true;
  }

private:
  void RegisterKeyframe(
      fuse_core::Transaction::SharedPtr transaction, ros::Time cur_time,
      fuse_variables::Orientation3DStamped::SharedPtr orienation,
      fuse_variables::Position3DStamped::SharedPtr position) {
    LMContainer lmc = this->tracker_->GetLandmarkContainer();
    /**
     * 1. get vector of id's in current image (cur_time)
     * 2. try to get the landmark, if failure then check if its in the previous
     * keyframe
     * 3. If landmark is successfully found (3d), add the visual constraint
     * 4. If it has not been found, triangulate it with the previous keyframe
     * and add constraints to both
     */
  }
  /**
   * @brief This function will search for untriangulated landmarks in the
   * current keyframe, triangulate them and add the position variables and
   * required constraints to the transaction
   * @param transaction
   */
  void AddNewLandmarks(fuse_core::Transaction::SharedPtr transaction) {
    std::vector<beam_cv::FeatureTrack> track =
        tracker_->GetTracks(img_num_ - 1);
    Eigen::Matrix4d T =
        this->FusePoseToMatrix(cur_kf_position_, cur_kf_orientation_);
    // add constraints for features not already in graph
    std::vector<uint64_t> lm_ids;
    std::vector<Eigen::Vector2d> pixel_v;
    for (auto& feature : track) {
      for (auto& measurement : feature) {
        if (measurement.image == img_num_ - 1 &&
            landmark_positions_.find(measurement.landmark_id) ==
                landmark_positions_.end()) {
          lm_ids.push_back(measurement.landmark_id);
          pixel_v.push_back(measurement.value);
        }
      }
    }
    uint64_t N = lm_ids.size();
    for (int i = 0; i < N; i++) {
      uint64_t id = lm_ids[i];
      // find measurement of same landmark in a different image
      bool feature_triangulated = false;
      for (auto& feature : track) {
        if (!feature_triangulated) {
          for (auto& measurement : feature) {
            if (measurement.image != img_num_ - 1 &&
                measurement.landmark_id == id) {
              // get pixel correspondence
              Eigen::Vector2d corr_pixel = measurement.value;
              // get position
              fuse_variables::Position3DStamped::SharedPtr corr_position =
                  this->GetCameraPosition(measurement.time_point);
              // get orientation from graph
              fuse_variables::Orientation3DStamped::SharedPtr corr_orientation =
                  this->GetCameraOrientation(measurement.time_point);
              // transform position + orientation into transformation matrix
              Eigen::Matrix4d corr_T =
                  this->FusePoseToMatrix(corr_position, corr_orientation);
              // triangulate landmark
              beam::opt<Eigen::Vector3d> point =
                  beam_cv::Triangulation::TriangulatePoint(
                      cam_model_, cam_model_, corr_T, T, corr_pixel.cast<int>(),
                      pixel_v[i].cast<int>());
              // build constraint
              if (point.has_value()) {
                Eigen::Vector3d point_T =
                    this->TransformPoint(point.value(), T);
                Eigen::Vector3d point_corr_T =
                    this->TransformPoint(point.value(), corr_T);

                if (point_T[2] > 0 && point_corr_T[2] > 0) {
                  fuse_variables::Position3D::SharedPtr landmark =
                      fuse_variables::Position3D::make_shared(
                          std::to_string(id).c_str());
                  landmark->x() = point.value()[0];
                  landmark->y() = point.value()[1];
                  landmark->z() = point.value()[2];
                  fuse_constraints::VisualConstraint::SharedPtr
                      vis_constraint2 =
                          fuse_constraints::VisualConstraint::make_shared(
                              source_, *corr_orientation, *corr_position,
                              *landmark, pixel_v[i], cam_model_);
                  // add landmark to local map
                  landmark_positions_[id] = landmark;
                  // add constraint to graph
                  transaction->addVariable(landmark);
                  transaction->addVariable(corr_orientation);
                  transaction->addVariable(corr_position);
                  transaction->addConstraint(vis_constraint2);
                  feature_triangulated = true;
                }
              }
            }
          }
        }
      }
    }
  }

  /**
   * @brief This function will search for already triangulated landmarks in the
   * current keyframe, and will add the required constraints
   * @param transaction
   */
  void AddExistingFeatureConstraints(
      fuse_core::Transaction::SharedPtr transaction) {
    std::vector<beam_cv::FeatureTrack> track =
        tracker_->GetTracks(img_num_ - 1);
    int count = 0;
    for (auto& feature : track) {
      for (auto& measurement : feature) {
        if (measurement.image == img_num_ - 1) {
          // retrieve landmark position from current graph
          fuse_variables::Position3D::SharedPtr landmark =
              this->GetLandmark(measurement.landmark_id);
          // if landmark exists in graph then create and add constraint to
          // transaction
          if (landmark != NULL) {
            Eigen::Vector2d pixel = measurement.value;
            // build constraint
            fuse_constraints::VisualConstraint::SharedPtr vis_constraint =
                fuse_constraints::VisualConstraint::make_shared(
                    source_, *cur_kf_orientation_, *cur_kf_position_, *landmark,
                    pixel, cam_model_);
            // add constraint to graph (cur pose, pixel, landmark point)
            transaction->addVariable(landmark);
            transaction->addConstraint(vis_constraint);
            count++;
          }
        }
      }
    }
  }

  /**
   * @brief This function will determine if the image at a specific time is a
   * keyframe by computing the parallax and computing translational movement
   * between it and the most recent keyframe
   * @param frame_timestamp of the image to compare to current keyframe with
   * @param position position of image to measure translational movement from
   * last keyframe
   */
  bool IsKeyframe(ros::Time frame_timestamp,
                  fuse_variables::Position3DStamped::SharedPtr position) {
    return true;
  }

  /**
   * @brief This function will compute the average parallax between the frame at
   * timestamp and the current keyframe
   * @param frame_timestamp of the image to compare to current keyframe with
   */
  float ComputeParallax(ros::Time frame_timestamp) { return 0.0; }

  /**
   * @brief Helper function to add constraints for already existing landmarks
   * @param track feature track of current image
   */
  fuse_variables::Orientation3DStamped::SharedPtr
      GetCameraOrientation(const ros::Time& stamp) {
    fuse_variables::Orientation3DStamped::SharedPtr corr_orientation =
        fuse_variables::Orientation3DStamped::make_shared();
    if (graph_initialized) {
      auto corr_orientation_uuid = fuse_core::uuid::generate(
          orientation_3d_stamped_type_, stamp, fuse_core::uuid::NIL);
      try {
        *corr_orientation =
            dynamic_cast<const fuse_variables::Orientation3DStamped&>(
                graph_->getVariable(corr_orientation_uuid));
        tracker_orientations_.erase(stamp.toSec());
      } catch (const std::out_of_range& oor) {
        corr_orientation = tracker_orientations_[stamp.toSec()];
      }
    } else {
      corr_orientation = tracker_orientations_[stamp.toSec()];
    }
    return corr_orientation;
  }

  /**
   * @brief Helper function to add constraints for already existing landmarks
   * @param track feature track of current image
   */
  fuse_variables::Position3DStamped::SharedPtr
      GetCameraPosition(const ros::Time& stamp) {
    fuse_variables::Position3DStamped::SharedPtr corr_position =
        fuse_variables::Position3DStamped::make_shared();
    if (graph_initialized) {
      auto corr_position_uuid = fuse_core::uuid::generate(
          position_3d_stamped_type_, stamp, fuse_core::uuid::NIL);
      try {
        *corr_position = dynamic_cast<const fuse_variables::Position3DStamped&>(
            graph_->getVariable(corr_position_uuid));
        tracker_positions_.erase(stamp.toSec());
      } catch (const std::out_of_range& oor) {
        corr_position = tracker_positions_[stamp.toSec()];
      }
    } else {
      corr_position = tracker_positions_[stamp.toSec()];
    }
    return corr_position;
  }

  /**
   * @brief Helper function to add constraints for already existing landmarks
   * @param track feature track of current image
   */
  fuse_variables::Position3D::SharedPtr GetLandmark(uint64_t landmark_id) {
    fuse_variables::Position3D::SharedPtr landmark =
        fuse_variables::Position3D::make_shared();
    if (graph_initialized) {
      auto landmark_uuid = fuse_core::uuid::generate(
          position_3d_type_, std::to_string(landmark_id).c_str());
      try {
        *landmark = dynamic_cast<const fuse_variables::Position3D&>(
            graph_->getVariable(landmark_uuid));
        landmark_positions_.erase(landmark_id);
      } catch (const std::out_of_range& oor) {
        landmark = landmark_positions_[landmark_id];
      }
    } else {
      landmark = landmark_positions_[landmark_id];
    }
    return landmark;
  }

  void SetPoseVariables(
      Eigen::Matrix4d& T_WORLD_SENSOR,
      fuse_variables::Orientation3DStamped::SharedPtr orientation,
      fuse_variables::Position3DStamped::SharedPtr position) {
    Eigen::Matrix3d R = T_WORLD_SENSOR.block<3, 3>(0, 0);
    Eigen::Quaterniond q(R);
    Eigen::Vector3d t = T_WORLD_SENSOR.block<3, 1>(0, 3).transpose();
    orientation->x() = q.x();
    orientation->y() = q.y();
    orientation->z() = q.z();
    orientation->w() = q.w();
    position->x() = t[0];
    position->y() = t[1];
    position->z() = t[2];
  }

  Eigen::Matrix4d FusePoseToMatrix(
      fuse_variables::Position3DStamped::SharedPtr position,
      fuse_variables::Orientation3DStamped::SharedPtr orientation) {
    Eigen::Quaterniond q(orientation->w(), orientation->x(), orientation->y(),
                         orientation->z());
    Eigen::Matrix3d R = q.normalized().toRotationMatrix();
    Eigen::Vector3d t(position->x(), position->y(), position->z());
    Eigen::Matrix4d T;
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = t.transpose();
    Eigen::Vector4d v{0, 0, 0, 1};
    T.row(3) = v;
    return T;
  }

  Eigen::Vector3d TransformPoint(Eigen::Vector3d point, Eigen::Matrix4d T) {
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    Eigen::Vector3d t = T.block<3, 1>(0, 3).transpose();
    return R * point + t;
  }

protected:
  // computer vision objects
  std::shared_ptr<beam_calibration::CameraModel> cam_model_;
  std::shared_ptr<beam_cv::Tracker> tracker_;

  // local maps for when graph doesnt have most up to date variables
  std::unordered_map<double, fuse_variables::Orientation3DStamped::SharedPtr>
      tracker_orientations_;
  std::unordered_map<double, fuse_variables::Position3DStamped::SharedPtr>
      tracker_positions_;
  std::unordered_map<uint64_t, fuse_variables::Position3D::SharedPtr>
      landmark_positions_;
  fuse_core::Graph::ConstSharedPtr graph_;
  bool graph_initialized = false;

  // keyframe info
  fuse_variables::Orientation3DStamped::SharedPtr cur_kf_orientation_;
  fuse_variables::Position3DStamped::SharedPtr cur_kf_position_;

  uint64_t img_num_ = 0;

  // variable types for uuid generations
  std::string source_ = "VO";
  std::string position_3d_type_ = "fuse_variables::Position3D";
  std::string position_3d_stamped_type_ = "fuse_variables::Position3DStamped";
  std::string orientation_3d_stamped_type_ =
      "fuse_variables::Orientation3DStamped";
};

}} // namespace beam_models::camera_to_camera
