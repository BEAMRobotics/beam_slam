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

VOInitializer::VOInitializer() : fuse_core::AsyncSensorModel(1) {}

void VOInitializer::onInit() {
  // load parameters from ros
  calibration_params_.loadFromROS();
  vo_initializer_params_.loadFromROS(private_node_handle_);

  // advertise init path publisher
  results_publisher_ =
      private_node_handle_.advertise<bs_common::InitializedPathMsg>("result",
                                                                    10);
  // subscribe to image topic
  image_subscriber_ =
      private_node_handle_.subscribe(vo_initializer_params_.image_topic, 100,
                                     &VOInitializer::processImage, this);

  // subscribe to reset topic
  reset_subscriber_ = private_node_handle_.subscribe(
      "/slam_reset", 1, &VOInitializer::processReset, this);

  // Load camera model and Create Map object
  cam_model_ = beam_calibration::CameraModel::Create(
      calibration_params_.cam_intrinsics_path);

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

void VOInitializer::processReset(const std_msgs::Bool::ConstPtr& msg) {
  // if a reset request is called then we set initialization to be incomplete
  // and we wipe memory
  if (msg->data == true) {
    initialization_complete_ = false;
    visual_map_->Clear();
    trajectory_.clear();
    times_.clear();
    output_times_.clear();
  }
}

void VOInitializer::processImage(const sensor_msgs::Image::ConstPtr& msg) {
  if (initialization_complete_) return;

  ros::Time cur_time = msg->header.stamp;
  // push image time into queue
  times_.push_back(cur_time);
  if (times_.size() > vo_initializer_params_.tracker_window_size) {
    times_.pop_front();
  }

  // add image to tracker
  tracker_->AddImage(beam_cv::OpenCVConversions::RosImgToMat(*msg), cur_time);

  // try to initialize
  if (times_.size() >= 10) {
    // Get matches between first and last image in the window
    std::vector<Eigen::Vector2i, beam::AlignVec2i> p1_v;
    std::vector<Eigen::Vector2i, beam::AlignVec2i> p2_v;
    std::vector<uint64_t> ids = tracker_->GetLandmarkIDsInImage(cur_time);
    double total_parallax = 0.0;
    double num_correspondences = 0.0;
    std::vector<uint64_t> matched_ids;
    for (auto& id : ids) {
      try {
        Eigen::Vector2i p1 = tracker_->Get(times_.front(), id).cast<int>();
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
              beam_cv::EstimatorMethod::SEVENPOINT, 50);

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
          // transform points into each camera frame
          Eigen::Vector4d pt_h;
          pt_h << points[i].value()[0], points[i].value()[1],
              points[i].value()[2], 1;
          Eigen::Vector4d pt_h_1 = T_world_c1.inverse() * pt_h,
                          pt_h_2 = T_world_c2.inverse() * pt_h;
          Eigen::Vector3d pt1 = pt_h_1.head(3) / pt_h_1(3);
          Eigen::Vector3d pt2 = pt_h_2.head(3) / pt_h_2(3);
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
          Eigen::Vector2d p1_d{p1_v[i][0], p1_v[i][1]};
          Eigen::Vector2d p2_d{p2_v[i][0], p2_v[i][1]};
          if (beam::distance(p1_rep, p1_d) > 5.0 &&
              beam::distance(p2_rep, p2_d) > 5.0) {
            // set outlier to have no value in the vector
            points[i].reset();
          } else {
            inliers++;
          }
        }
      }
      float inlier_ratio = (float)inliers / p1_v.size();
      if (inlier_ratio > 0.8) {
        ROS_INFO("Valid image pair. Parallax: %f, Inlier Ratio: %f. Attempting "
                 "VO Initialization.",
                 parallax, inlier_ratio);
        auto transaction = fuse_core::Transaction::make_shared();
        transaction->stamp(times_.front());

        // add poses to map
        visual_map_->AddCameraPose(T_world_c1, times_.front(), transaction);
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
          visual_map_->AddConstraint(times_.front(), id,
                                     tracker_->Get(times_.front(), id),
                                     transaction);
          visual_map_->AddConstraint(cur_time, id, tracker_->Get(cur_time, id),
                                     transaction);
        }

        output_times_.push_back(times_.front());
        // localize frames in between
        for (size_t i = 0; i < times_.size(); i += 5) {
          if (times_[i] == times_.front() || times_[i] == cur_time) continue;
          output_times_.push_back(times_[i]);
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
                Eigen::Vector2i pixeli =
                    tracker_->Get(times_[i], id).cast<int>();
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
          visual_map_->AddCameraPose(T_CAMERA_WORLD_est.inverse(), times_[i],
                                     transaction);

          // add visual constraints to
          for (auto& id : ids_in_frame) {
            visual_map_->AddConstraint(
                times_[i], id, tracker_->Get(times_[i], id), transaction);
          }
        }
        output_times_.push_back(cur_time);

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
        for (size_t i = 0; i < output_times_.size(); i++) {
          trajectory_.push_back(
              visual_map_->GetBaselinkPose(output_times_[i]).value());
        }
        ROS_INFO("VO initializer complete, publishing result.");
        PublishResults();

        // clean up memory
        visual_map_->Clear();
        trajectory_.clear();
        times_.clear();
        output_times_.clear();
        initialization_complete_ = true;
      }
    }
  }
}

void VOInitializer::PublishResults() {
  bs_common::InitializedPathMsg msg;

  for (uint32_t i = 0; i < trajectory_.size(); i++) {
    Eigen::Matrix4d T = trajectory_[i];

    std_msgs::Header header;
    header.frame_id = extrinsics_.GetBaselinkFrameId();
    header.seq = i;
    header.stamp = output_times_[i];

    geometry_msgs::Point position;
    position.x = T(0, 3);
    position.y = T(1, 3);
    position.z = T(2, 3);

    Eigen::Matrix3d R = T.block(0, 0, 3, 3);
    Eigen::Quaterniond q(R);

    geometry_msgs::Quaternion orientation;
    orientation.x = q.x();
    orientation.y = q.y();
    orientation.z = q.z();
    orientation.w = q.w();

    geometry_msgs::PoseStamped pose;
    pose.header = header;
    pose.pose.position = position;
    pose.pose.orientation = orientation;
    msg.poses.push_back(pose);
  }
  results_publisher_.publish(msg);
}

} // namespace bs_models