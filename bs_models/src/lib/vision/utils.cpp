#include <bs_models/vision/utils.h>

#include <beam_cv/geometry/AbsolutePoseEstimator.h>
#include <beam_cv/geometry/RelativePoseEstimator.h>
#include <beam_cv/geometry/Triangulation.h>
#include <beam_utils/utils.h>
#include <bs_models/vision/visual_map.h>
#include <fuse_graphs/hash_graph.h>

#include <algorithm>

namespace bs_models { namespace vision {

std::map<uint64_t, Eigen::Matrix4d> computePathWithVision(
    const std::shared_ptr<beam_calibration::CameraModel>& camera_model,
    const std::shared_ptr<beam_containers::LandmarkContainer>& landmark_container,
    const Eigen::Matrix4d& T_camera_baselink, double max_optimization_time) {
  const auto img_times_set = landmark_container->GetMeasurementTimes();
  std::vector<uint64_t> img_times;
  std::for_each(img_times_set.begin(), img_times_set.end(),
                [&](const uint64_t& time) { img_times.push_back(time); });
  const int num_images = img_times.size();
  if (num_images < 2) { return {}; }

  // Get matches between first and last image in the window
  ros::Time first_time = beam::NSecToRos(*img_times.begin());
  ros::Time last_time = beam::NSecToRos(*img_times.rbegin());
  std::vector<Eigen::Vector2i, beam::AlignVec2i> first_landmarks;
  std::vector<Eigen::Vector2i, beam::AlignVec2i> last_landmarks;
  std::vector<uint64_t> ids = landmark_container->GetLandmarkIDsInImage(last_time);
  std::vector<uint64_t> matched_ids;
  for (auto& id : ids) {
    try {
      first_landmarks.push_back(landmark_container->GetValue(first_time, id).cast<int>());
      last_landmarks.push_back(landmark_container->GetValue(last_time, id).cast<int>());
      matched_ids.push_back(id);
    } catch (const std::out_of_range& oor) {}
  }

  // Compute relative pose between first and last
  beam::opt<Eigen::Matrix4d> T_last_first = beam_cv::RelativePoseEstimator::RANSACEstimator(
      camera_model, camera_model, first_landmarks, last_landmarks,
      beam_cv::EstimatorMethod::SEVENPOINT, 100);
  if (!T_last_first.has_value()) { return {}; }

  Eigen::Matrix4d T_world_first_frame = T_camera_baselink.inverse();
  Eigen::Matrix4d T_world_last_frame = T_world_first_frame * T_last_first.value().inverse();

  // Triangulate points
  std::vector<beam::opt<Eigen::Vector3d>> points = beam_cv::Triangulation::TriangulatePoints(
      camera_model, camera_model, T_world_first_frame.inverse(), T_world_last_frame.inverse(),
      first_landmarks, last_landmarks);

  // Determine inliers of triangulated points
  int inliers = 0;
  for (size_t i = 0; i < points.size(); i++) {
    if (points[i].has_value()) {
      Eigen::Vector3d p = points[i].value();
      // transform points into each camera frame
      Eigen::Vector3d pt1 = (T_world_first_frame.inverse() * p.homogeneous()).hnormalized(),
                      pt2 = (T_world_last_frame.inverse() * p.homogeneous()).hnormalized();
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
      Eigen::Vector2d p1_d = first_landmarks[i].cast<double>();
      Eigen::Vector2d p2_d = last_landmarks[i].cast<double>();
      if (beam::distance(p1_rep, p1_d) > 5.0 && beam::distance(p2_rep, p2_d) > 5.0) {
        // set outlier to have no value in the vector
        points[i].reset();
      } else {
        inliers++;
      }
    }
  }

  // If not enough inliers, return
  float inlier_ratio = (float)inliers / first_landmarks.size();
  if (inlier_ratio < 0.8) { return {}; }

  ROS_INFO("Valid image pair. Inlier Ratio: %f. Attempting VO Path Estimation.", inlier_ratio);

  // Perform SFM
  auto visual_graph = std::make_shared<fuse_graphs::HashGraph>();
  auto visual_map = std::make_shared<vision::VisualMap>(camera_model);

  // Add initial poses, landmarks and constraints to graph
  auto initial_transaction = fuse_core::Transaction::make_shared();
  initial_transaction->stamp(first_time);
  visual_map->AddCameraPose(T_world_first_frame, first_time, initial_transaction);
  visual_map->AddCameraPose(T_world_last_frame, last_time, initial_transaction);

  for (size_t i = 0; i < points.size(); i++) {
    if (!points[i].has_value()) { continue; }
    const auto landmark_id = matched_ids[i];
    visual_map->AddLandmark(points[i].value(), landmark_id, initial_transaction);

    Eigen::Vector2d first_pixel = landmark_container->GetValue(first_time, landmark_id);
    Eigen::Vector2d last_pixel = landmark_container->GetValue(last_time, landmark_id);
    Eigen::Vector2i tmp;
    if (camera_model->UndistortPixel(first_pixel.cast<int>(), tmp) &&
        camera_model->UndistortPixel(last_pixel.cast<int>(), tmp)) {
      visual_map->AddVisualConstraint(first_time, landmark_id, first_pixel, initial_transaction);
      visual_map->AddVisualConstraint(last_time, landmark_id, last_pixel, initial_transaction);
    }
  }
  visual_graph->update(*initial_transaction);
  visual_map->UpdateGraph(visual_graph);

  // Localize frames in between and add variables to graph
  for (int i = 0; i < num_images; i++) {
    ros::Time timestamp = beam::NSecToRos(img_times[i]);
    if (timestamp == first_time || timestamp == last_time) continue;

    auto transaction = fuse_core::Transaction::make_shared();
    transaction->stamp(timestamp);
    // get 2d-3d correspondences
    std::vector<Eigen::Vector2i, beam::AlignVec2i> pixels;
    std::vector<Eigen::Vector3d, beam::AlignVec3d> points;
    std::vector<uint64_t> ids_in_frame;
    for (auto& id : matched_ids) {
      fuse_variables::Point3DLandmark::SharedPtr lm = visual_map->GetLandmark(id);
      if (lm) {
        try {
          Eigen::Vector3d point = lm->point();
          Eigen::Vector2i pixeli = landmark_container->GetValue(timestamp, id).cast<int>();
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
    visual_map->AddCameraPose(T_CAMERA_WORLD_est.inverse(), timestamp, transaction);

    // add visual constraints to
    for (auto& id : ids_in_frame) {
      Eigen::Vector2d measurement = landmark_container->GetValue(timestamp, id);
      Eigen::Vector2i tmp;
      if (camera_model->UndistortPixel(measurement.cast<int>(), tmp)) {
        visual_map->AddVisualConstraint(timestamp, id, measurement, transaction);
      }
    }
    // add transaction to graph and notify visual map
    visual_graph->update(*transaction);
    visual_map->UpdateGraph(visual_graph);
  }

  // optimize graph
  ceres::Solver::Options options;
  std::cout << "Optimizing" << std::endl;
  visual_graph->optimize();
  visual_map->UpdateGraph(visual_graph);

  // return result
  pcl::PointCloud<pcl::PointXYZRGB> frame_cloud;

  std::map<uint64_t, Eigen::Matrix4d> init_path;
  for (const auto& time : img_times) {
    const auto ros_time = beam::NSecToRos(time);
    const auto pose = visual_map->GetBaselinkPose(ros_time);
    if (pose.has_value()) {
      std::cout << "-----" << std::endl;
      std::cout << ros_time << std::endl;
      std::cout << pose.value().inverse() << std::endl;
      frame_cloud = beam::AddFrameToCloud(frame_cloud, pose.value().inverse(), 0.001);
      init_path[time] = pose.value().inverse();
    }
  }

  if (!beam::SavePointCloud<pcl::PointXYZRGB>("/userhome/frames.pcd", frame_cloud,
                                              beam::PointCloudFileType::PCDBINARY)) {
    BEAM_ERROR("Unable to save cloud.");
  }
  return init_path;
}

}} // namespace bs_models::vision