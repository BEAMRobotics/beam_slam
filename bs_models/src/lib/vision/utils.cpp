#include <bs_models/vision/utils.h>

#include <beam_cv/geometry/AbsolutePoseEstimator.h>
#include <beam_cv/geometry/RelativePoseEstimator.h>
#include <beam_cv/geometry/Triangulation.h>
#include <bs_models/vision/visual_map.h>
#include <fuse_graphs/hash_graph.h>

namespace bs_models { namespace vision {

std::map<uint64_t, Eigen::Matrix4d> computePathWithVision(
    const std::shared_ptr<beam_calibration::CameraModel>& camera_model,
    const std::shared_ptr<beam_containers::LandmarkContainer>& landmark_container,
    const Eigen::Matrix4d& T_camera_baselink, double max_optimization_time, int images_to_skip) {
  const auto img_times_set = landmark_container->GetMeasurementTimes();
  std::vector<uint64_t> img_times;
  std::transform(img_times_set.begin(), img_times_set.end(), img_times.begin(),
                 [&](const auto& time) { return time; });
  const int num_images = img_times.size();
  if (num_images < 2) { return {}; }
  // Get matches between first and last image in the window
  ros::Time first_time(0, img_times[0]);
  ros::Time last_time(0, img_times[img_times.size() - 1]);
  std::vector<Eigen::Vector2i, beam::AlignVec2i> p1_v;
  std::vector<Eigen::Vector2i, beam::AlignVec2i> p2_v;
  std::vector<uint64_t> ids = landmark_container->GetLandmarkIDsInImage(last_time);
  std::vector<uint64_t> matched_ids;
  for (auto& id : ids) {
    try {
      p1_v.push_back(landmark_container->GetValue(first_time, id).cast<int>());
      p2_v.push_back(landmark_container->GetValue(last_time, id).cast<int>());
      matched_ids.push_back(id);
    } catch (const std::out_of_range& oor) {}
  }
  // compute relative pose
  beam::opt<Eigen::Matrix4d> T_c2_c1 = beam_cv::RelativePoseEstimator::RANSACEstimator(
      camera_model, camera_model, p1_v, p2_v, beam_cv::EstimatorMethod::SEVENPOINT, 100);
  if (!T_c2_c1.has_value()) { return {}; }

  auto visual_graph = std::make_shared<fuse_graphs::HashGraph>();
  auto visual_map = std::make_shared<vision::VisualMap>(camera_model);

  Eigen::Matrix4d T_world_c1 = T_camera_baselink.inverse();
  Eigen::Matrix4d T_world_c2 = T_world_c1 * T_c2_c1.value().inverse();

  // triangulate points
  std::vector<beam::opt<Eigen::Vector3d>> points = beam_cv::Triangulation::TriangulatePoints(
      camera_model, camera_model, T_world_c1.inverse(), T_world_c2.inverse(), p1_v, p2_v);

  // determine inliers
  int inliers = 0;
  for (size_t i = 0; i < points.size(); i++) {
    if (points[i].has_value()) {
      Eigen::Vector3d p = points[i].value();
      // transform points into each camera frame
      Eigen::Vector3d pt1 = (T_world_c1.inverse() * p.homogeneous()).hnormalized(),
                      pt2 = (T_world_c2.inverse() * p.homogeneous()).hnormalized();
      // reproject triangulated points into each frame
      bool in_image1 = false, in_image2 = false;
      Eigen::Vector2d p1_rep, p2_rep;
      if (!camera_model->ProjectPoint(pt1, p1_rep, in_image1) ||
          !camera_model->ProjectPoint(pt2, p2_rep, in_image2)) {
        continue;
      } else if (!in_image1 || !in_image2) {
        continue;
      }
      // compute distance to actual pixel
      Eigen::Vector2d p1_d = p1_v[i].cast<double>();
      Eigen::Vector2d p2_d = p2_v[i].cast<double>();
      if (beam::distance(p1_rep, p1_d) > 5.0 && beam::distance(p2_rep, p2_d) > 5.0) {
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
    ROS_INFO("Valid image pair. Inlier Ratio: %f. Attempting VO Path Estimation.", inlier_ratio);
    auto transaction = fuse_core::Transaction::make_shared();
    transaction->stamp(first_time);

    // add poses to map
    visual_map->AddCameraPose(T_world_c1, first_time, transaction);
    visual_map->AddCameraPose(T_world_c2, last_time, transaction);

    // add landmarks to map
    for (size_t i = 0; i < points.size(); i++) {
      if (points[i].has_value()) {
        visual_map->AddLandmark(points[i].value(), matched_ids[i], transaction);
      }
    }

    // add visual constraints to map
    for (auto& id : matched_ids) {
      Eigen::Vector2d cur_pixel = landmark_container->GetValue(last_time, id);
      Eigen::Vector2d first_pixel = landmark_container->GetValue(first_time, id);
      Eigen::Vector2i tmp;
      if (camera_model->UndistortPixel(first_pixel.cast<int>(), tmp) &&
          camera_model->UndistortPixel(cur_pixel.cast<int>(), tmp)) {
        visual_map->AddVisualConstraint(first_time, id, first_pixel, transaction);
        visual_map->AddVisualConstraint(last_time, id, cur_pixel, transaction);
      }
    }

    // localize frames in between
    std::vector<ros::Time> kf_times;
    for (int i = 0; i < num_images; i += images_to_skip) {
      ros::Time kf_time(0, img_times[i]);
      kf_times.push_back(kf_time);
      if (kf_time == first_time || kf_time == last_time) continue;
      // get 2d-3d correspondences
      std::vector<Eigen::Vector2i, beam::AlignVec2i> pixels;
      std::vector<Eigen::Vector3d, beam::AlignVec3d> points;
      std::vector<uint64_t> ids_in_frame;
      for (auto& id : matched_ids) {
        fuse_variables::Point3DLandmark::SharedPtr lm = visual_map->GetLandmark(id);
        if (lm) {
          // we wrap this is a try catch block in the off chance an id in
          // matched ids isnt in the current frame, however with the way the
          // tracker works, all matched ids will be in these frames
          try {
            Eigen::Vector3d point(lm->x(), lm->y(), lm->z());
            Eigen::Vector2i pixeli = landmark_container->GetValue(kf_time, id).cast<int>();
            pixels.push_back(pixeli);
            points.push_back(point);
            ids_in_frame.push_back(id);
          } catch (const std::out_of_range& oor) {}
        }
      }
      // localize with correspondences
      Eigen::Matrix4d T_CAMERA_WORLD_est =
          beam_cv::AbsolutePoseEstimator::RANSACEstimator(camera_model, pixels, points, 30);

      // add pose to graph
      visual_map->AddCameraPose(T_CAMERA_WORLD_est.inverse(), kf_time, transaction);

      // add visual constraints to
      for (auto& id : ids_in_frame) {
        Eigen::Vector2d measurement = landmark_container->GetValue(kf_time, id);
        Eigen::Vector2i tmp;
        if (camera_model->UndistortPixel(measurement.cast<int>(), tmp)) {
          visual_map->AddVisualConstraint(kf_time, id, measurement, transaction);
        }
      }
    }

    // modify graph with these additions
    visual_graph->update(*transaction);

    // optimize graph
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = false;
    options.num_threads = 6;
    options.num_linear_solver_threads = 6;
    options.minimizer_type = ceres::TRUST_REGION;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.preconditioner_type = ceres::SCHUR_JACOBI;
    options.max_solver_time_in_seconds = max_optimization_time;
    options.max_num_iterations = 100;
    visual_graph->optimize(options);
    visual_map->UpdateGraph(visual_graph);

    // return result
    std::map<uint64_t, Eigen::Matrix4d> init_path;
    for (const auto& kf : kf_times) {
      init_path[kf.toNSec()] = visual_map->GetBaselinkPose(kf).value();
    }
    return init_path;
  }
}

}} // namespace bs_models::vision