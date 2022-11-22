#include <bs_models/slam_initializers/vo_initializer.h>

#include <pluginlib/class_list_macros.h>

#include <beam_cv/OpenCVConversions.h>
#include <beam_cv/Utils.h>
#include <beam_cv/detectors/Detectors.h>
#include <beam_cv/geometry/AbsolutePoseEstimator.h>
#include <beam_cv/geometry/RelativePoseEstimator.h>
#include <beam_cv/geometry/Triangulation.h>
#include <beam_utils/math.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(bs_models::VOInitializer, fuse_core::SensorModel)

namespace bs_models {

void VOInitializer::onInit() {
  // load parameters from ros
  calibration_params_.loadFromROS();
  vo_initializer_params_.loadFromROS(private_node_handle_);

  // subscribe to image topic
  image_subscriber_ =
      private_node_handle_.subscribe(vo_initializer_params_.image_topic, 100,
                                     &VOInitializer::processImage, this);

  // Load camera model and Create Map object
  cam_model_ = beam_calibration::CameraModel::Create(
      calibration_params_.cam_intrinsics_path);
  cam_model_->InitUndistortMap();

  // create optimzation graph
  local_graph_ = std::make_shared<fuse_graphs::HashGraph>();

  // create visual map
  visual_map_ = std::make_shared<vision::VisualMap>(cam_model_);

  // Initialize detector
  std::shared_ptr<beam_cv::Detector> detector = beam_cv::Detector::Create(
      beam_cv::DetectorTypeStringMap[vo_initializer_params_.detector],
      vo_initializer_params_.detector_config);

  // Initialize tracker
  beam_cv::KLTracker::Params tracker_params;
  tracker_params.LoadFromJson(vo_initializer_params_.tracker_config);
  tracker_ = std::make_shared<beam_cv::KLTracker>(
      tracker_params, detector, nullptr,
      vo_initializer_params_.tracker_window_size);
}
void VOInitializer::processImage(const sensor_msgs::Image::ConstPtr& msg) {
  if (initialization_complete_) {
    image_subscriber_.shutdown();
    return;
  }

  ros::Time cur_time = msg->header.stamp;
  // push image time into queue
  kf_times_.push_back(cur_time);
  if (kf_times_.size() > vo_initializer_params_.tracker_window_size) {
    kf_times_.pop_front();
  }

  // add image to tracker
  tracker_->AddImage(beam_cv::OpenCVConversions::RosImgToMat(*msg), cur_time);

  // try to initialize
  if (kf_times_.size() >= 10) {
    // Get matches between first and last image in the window
    std::vector<Eigen::Vector2i, beam::AlignVec2i> p1_v;
    std::vector<Eigen::Vector2i, beam::AlignVec2i> p2_v;
    std::vector<uint64_t> ids = tracker_->GetLandmarkIDsInImage(cur_time);
    double total_parallax = 0.0;
    double num_correspondences = 0.0;
    std::vector<uint64_t> matched_ids;
    for (auto& id : ids) {
      try {
        Eigen::Vector2i p1 = tracker_->Get(kf_times_.front(), id).cast<int>();
        Eigen::Vector2i p2 = tracker_->Get(cur_time, id).cast<int>();
        p1_v.push_back(p1);
        p2_v.push_back(p2);
        matched_ids.push_back(id);
        double d = beam::distance(p1, p2);
        total_parallax += d;
        num_correspondences += 1.0;
      } catch (const std::out_of_range& oor) {}
    }
    double parallax = total_parallax / num_correspondences;

    // if parallax is good then try to initialize
    if (parallax > vo_initializer_params_.parallax) {
      // compute relative pose
      beam::opt<Eigen::Matrix4d> T_c2_c1 =
          beam_cv::RelativePoseEstimator::RANSACEstimator(
              cam_model_, cam_model_, p1_v, p2_v,
              beam_cv::EstimatorMethod::SEVENPOINT, 10);

      // compute world poses
      if (!extrinsics_.GetT_CAMERA_BASELINK(T_cam_baselink_)) {
        ROS_ERROR("Unable to get camera to baselink transform.");
        return;
      }
      Eigen::Matrix4d T_world_c1 = T_cam_baselink_.inverse();
      Eigen::Matrix4d T_world_c2 = T_world_c1 * T_c2_c1.value().inverse();

      // triangulate points
      std::vector<beam::opt<Eigen::Vector3d>> points =
          beam_cv::Triangulation::TriangulatePoints(
              cam_model_, cam_model_, T_world_c1.inverse(),
              T_world_c2.inverse(), p1_v, p2_v);

      // determine inliers
      int inliers = 0;
      for (size_t i = 0; i < points.size(); i++) {
        if (points[i].has_value()) {
          Eigen::Vector3d p = points[i].value();
          // transform points into each camera frame
          Eigen::Vector3d
              pt1 = (T_world_c1.inverse() * p.homogeneous()).hnormalized(),
              pt2 = (T_world_c2.inverse() * p.homogeneous()).hnormalized();
          // reproject triangulated points into each frame
          bool in_image1 = false, in_image2 = false;
          Eigen::Vector2d p1_rep, p2_rep;
          if (!cam_model_->ProjectPoint(pt1, p1_rep, in_image1) ||
              !cam_model_->ProjectPoint(pt2, p2_rep, in_image2)) {
            continue;
          } else if (!in_image1 || !in_image2) {
            continue;
          }
          // compute distance to actual pixel
          Eigen::Vector2d p1_d = p1_v[i].cast<double>();
          Eigen::Vector2d p2_d = p2_v[i].cast<double>();
          if (beam::distance(p1_rep, p1_d) > 5.0 &&
              beam::distance(p2_rep, p2_d) > 5.0) {
            // set outlier to have no value in the vector
            points[i].reset();
          } else {
            inliers++;
          }
        }
      }
      // initialize sfm if we have enough inliers
      float inlier_ratio = (float)inliers / p1_v.size();
      if (inlier_ratio > 0.8) {
        ROS_INFO("Valid image pair. Parallax: %f, Inlier Ratio: %f. Attempting "
                 "VO Initialization.",
                 parallax, inlier_ratio);
        auto transaction = fuse_core::Transaction::make_shared();
        transaction->stamp(kf_times_.front());

        // add poses to map
        visual_map_->AddCameraPose(T_world_c1, kf_times_.front(), transaction);
        visual_map_->AddCameraPose(T_world_c2, cur_time, transaction);

        // add landmarks to map
        for (size_t i = 0; i < points.size(); i++) {
          if (points[i].has_value()) {
            visual_map_->AddLandmark(points[i].value(), matched_ids[i],
                                     transaction);
          }
        }

        // add visual constraints to map
        for (auto& id : matched_ids) {
          Eigen::Vector2d cur_pixel = tracker_->Get(cur_time, id);
          Eigen::Vector2d first_pixel = tracker_->Get(kf_times_.front(), id);
          Eigen::Vector2i tmp;
          if (cam_model_->UndistortPixel(first_pixel.cast<int>(), tmp) &&
              cam_model_->UndistortPixel(cur_pixel.cast<int>(), tmp)) {
            visual_map_->AddVisualConstraint(kf_times_.front(), id, first_pixel,
                                             transaction);
            visual_map_->AddVisualConstraint(cur_time, id, cur_pixel,
                                             transaction);
          }
        }

        // localize frames in between
        for (size_t i = 0; i < kf_times_.size(); i += 5) {
          ros::Time kf_time = kf_times_[i];
          times_.push_back(kf_time);
          if (kf_time == kf_times_.front() || kf_time == cur_time) continue;
          // get 2d-3d correspondences
          std::vector<Eigen::Vector2i, beam::AlignVec2i> pixels;
          std::vector<Eigen::Vector3d, beam::AlignVec3d> points;
          std::vector<uint64_t> ids_in_frame;
          for (auto& id : matched_ids) {
            fuse_variables::Point3DLandmark::SharedPtr lm =
                visual_map_->GetLandmark(id);
            if (lm) {
              // we wrap this is a try catch block in the off chance an id in
              // matched ids isnt in the current frame, however with the way the
              // tracker works, all matched ids will be in these frames
              try {
                Eigen::Vector3d point(lm->x(), lm->y(), lm->z());
                Eigen::Vector2i pixeli = tracker_->Get(kf_time, id).cast<int>();
                pixels.push_back(pixeli);
                points.push_back(point);
                ids_in_frame.push_back(id);
              } catch (const std::out_of_range& oor) {}
            }
          }
          // localize with correspondences
          Eigen::Matrix4d T_CAMERA_WORLD_est =
              beam_cv::AbsolutePoseEstimator::RANSACEstimator(
                  cam_model_, pixels, points, 30);

          // add pose to graph
          visual_map_->AddCameraPose(T_CAMERA_WORLD_est.inverse(), kf_time,
                                     transaction);

          // add visual constraints to
          for (auto& id : ids_in_frame) {
            Eigen::Vector2d measurement = tracker_->Get(kf_time, id);
            Eigen::Vector2i tmp;
            if (cam_model_->UndistortPixel(measurement.cast<int>(), tmp)) {
              visual_map_->AddVisualConstraint(kf_time, id, measurement,
                                               transaction);
            }
          }
        }

        // modify graph with these additions
        local_graph_->update(*transaction);

        // optimize graph
        ceres::Solver::Options options;
        options.minimizer_progress_to_stdout = false;
        options.num_threads = 6;
        options.num_linear_solver_threads = 6;
        options.minimizer_type = ceres::TRUST_REGION;
        options.linear_solver_type = ceres::SPARSE_SCHUR;
        options.preconditioner_type = ceres::SCHUR_JACOBI;
        options.max_solver_time_in_seconds = 0.5;
        options.max_num_iterations = 10;
        local_graph_->optimize(options);

        // publish result
        visual_map_->UpdateGraph(local_graph_);
        for (size_t i = 0; i < times_.size(); i++) {
          trajectory_.push_back(
              visual_map_->GetBaselinkPose(times_[i]).value());
        }
        ROS_INFO("VO initializer complete, publishing result.");
        PublishResults();

        // clean up memory
        visual_map_->Clear();
        trajectory_.clear();
        kf_times_.clear();
        times_.clear();
        initialization_complete_ = true;
      }
    }
  }
}
} // namespace bs_models
